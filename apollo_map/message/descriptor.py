"""
Cached descriptor bundles and dynamically generated protobuf message classes.

Author: Lejin Li, Kyushu University & National Institute of Informatics, Japan
Contact: li.lejin.147@s.kyushu-u.ac.jp

For more information about the serialization of protobuf messages, see:
https://buf.build/docs/reference/compilation-and-descriptors/#descriptor-production
https://googleapis.dev/python/protobuf/latest/google/protobuf/descriptor.html
"""

from pathlib import Path
from threading import RLock
from typing import Tuple, Set, Iterator, Dict, Type

from google.protobuf.descriptor_pb2 import FileDescriptorSet
from google.protobuf.message import Message
from google.protobuf.descriptor_pool import DescriptorPool

try:
    from google.protobuf.message_factory import GetMessageClass
except ImportError:  # protobuf 3.20, which commonroad-io pins
    GetMessageClass = None


def _message_class(pool: DescriptorPool, descriptor):
    """Build a message class from a pool descriptor.

    protobuf 4+ exposes ``GetMessageClass``. commonroad-io requires
    protobuf 3.20.2, where the same step is ``MessageFactory.GetPrototype``.
    """
    if GetMessageClass is not None:
        return GetMessageClass(descriptor)
    from google.protobuf.message_factory import MessageFactory

    return MessageFactory(pool).GetPrototype(descriptor)


class _DescriptorBundleCache(type):
    """
    Publish a cached instance only after its initialization succeeds.
    """

    _instances = {}
    _lock = RLock()

    def __call__(cls, descriptor_path: str | Path):
        path = Path(descriptor_path).expanduser().resolve()
        key = (cls, path)
        with cls._lock:
            if key not in cls._instances:
                cls._instances[key] = super().__call__(path)
            return cls._instances[key]


class DescriptorBundle(metaclass=_DescriptorBundleCache):
    """
    Registry of protobuf message definitions and lazily generated classes.

    A DescriptorBundle represents one serialized protobuf descriptor bundle.
    Different descriptor files can be loaded simultaneously, which makes this
    class suitable for comparing protobuf definitions across different Apollo
    versions.

    Construction reuses the same instance for the same resolved path within
    this process. Descriptors and message names are loaded during construction;
    message classes are generated and cached on demand. Descriptor files are
    treated as immutable once loaded; changes on disk are not reloaded.

    :param str | Path descriptor_path: Path to the serialized FileDescriptorSet
    """

    def __init__(self, descriptor_path: str | Path) -> None:
        """
        Load descriptors into an isolated pool and index message names.

        :param str | Path descriptor_path: Path to the serialized FileDescriptorSet
        :raises FileNotFoundError: If the path does not refer to a file
        :raises ValueError: If the descriptor dependencies contain a cycle
        :raises google.protobuf.message.DecodeError: If the bytes cannot be parsed
            as a FileDescriptorSet
        :raises TypeError: If the descriptor pool rejects a file definition
        """
        self._descriptor_path = Path(descriptor_path).expanduser().resolve()

        if not self._descriptor_path.is_file():
            raise FileNotFoundError(
                f"Descriptor file does not exist: {self._descriptor_path}"
            )

        self._descriptor_set = self._load_descriptor_set()
        self._pool = DescriptorPool()
        self._messages: Dict[str, Type[Message]] = {}
        self._names: Dict[str, list[str]] = {}
        self._full_names: Dict[str, None] = {}
        files = {file.name: file for file in self._descriptor_set.file}
        visiting = set()

        def add_file(name: str) -> None:
            if name not in files:
                return
            if name in visiting:
                raise ValueError(f"Cyclic descriptor dependency: {name!r}")
            visiting.add(name)
            file = files[name]
            for dependency in file.dependency:
                add_file(dependency)
            self._pool.Add(file)
            visiting.remove(name)
            del files[name]

        def index_messages(messages, prefix: str) -> None:
            for descriptor in messages:
                full_name = f"{prefix}.{descriptor.name}" if prefix else descriptor.name
                self._full_names[full_name] = None
                self._names.setdefault(descriptor.name, []).append(full_name)
                index_messages(descriptor.nested_type, full_name)

        for file in self._descriptor_set.file:
            add_file(file.name)
            index_messages(file.message_type, file.package)

    def _load_descriptor_set(self) -> FileDescriptorSet:
        """
        Load the FileDescriptorSet from disk.

        :returns: The parsed FileDescriptorSet
        :rtype: FileDescriptorSet
        """
        descriptor_set = FileDescriptorSet()
        descriptor_set.ParseFromString(self._descriptor_path.read_bytes())
        return descriptor_set

    @property
    def descriptor_path(self) -> Path:
        """
        Return the resolved absolute path used to cache this bundle.

        :returns: Path to the serialized FileDescriptorSet
        :rtype: Path
        """
        return self._descriptor_path

    @property
    def descriptor_set(self) -> FileDescriptorSet:
        """
        Return the internal FileDescriptorSet, not a copy.

        Treat the returned object as read-only: mutations do not update the
        descriptor pool, message-name indexes, or cached message classes.

        :returns: The parsed FileDescriptorSet
        :rtype: FileDescriptorSet
        """
        return self._descriptor_set

    def serialized_file_descriptors(self) -> Tuple[bytes, ...]:
        """
        Return serialized FileDescriptorProto messages in input bundle order.

        The returned sequence is not reordered by dependency.

        :returns: A tuple containing one serialized FileDescriptorProto for
                  each file in the descriptor set
        :rtype: Tuple[bytes, ...]
        """
        return tuple(
            file_descriptor.SerializeToString()
            for file_descriptor in self._descriptor_set.file
        )

    def messages(self) -> Dict[str, Type[Message]]:
        """
        Generate and cache all message classes, including nested messages.

        Return a new dictionary keyed by fully qualified names. Classes already
        cached in this bundle are reused.

        :returns: A mapping from fully qualified protobuf message names to
                  their dynamically generated Python classes
        :rtype: Dict[str, Type[Message]]
        """
        return {name: self.message(name) for name in self._full_names}

    def message(self, full_name: str) -> Type[Message]:
        """
        Resolve and cache a message class by full name or unique short name.

        :param str full_name: Fully qualified protobuf message name or unique short name
        :returns: The corresponding protobuf message class
        :rtype: Type[Message]
        :raises KeyError: If the name is missing or ambiguous in this bundle
        """
        if full_name not in self._full_names:
            matches = self._names.get(full_name, [])
            if len(matches) > 1:
                raise KeyError(
                    f"Ambiguous message name {full_name!r}; use one of: "
                    + ", ".join(matches)
                )
            if matches:
                full_name = matches[0]
        if full_name not in self._full_names:
            raise KeyError(
                f"{full_name!r} is not defined in "
                f"{self._descriptor_path.name!r}"
            )
        if full_name not in self._messages:
            self._messages[full_name] = _message_class(
                self._pool, self._pool.FindMessageTypeByName(full_name)
            )
        return self._messages[full_name]

    def message_names(self) -> Set[str]:
        """
        Return all fully qualified names, including nested messages.

        This does not generate message classes.

        :returns: A set of fully qualified protobuf message names
        :rtype: Set[str]
        """
        return set(self._full_names)

    def has_message(self, full_name: str) -> bool:
        """
        Check whether a full name or unique short name exists in the bundle.

        :param str full_name: Fully qualified protobuf message name or short name
        :returns: True if the name resolves uniquely; False if missing or ambiguous
        :rtype: bool
        """
        return full_name in self._full_names or len(self._names.get(full_name, [])) == 1

    def __getitem__(self, full_name: str) -> Type[Message]:
        """
        Return a message class using dictionary-style access.
        E.g., Header = bundle["apollo.common.Header"]

        :param str full_name: Fully qualified protobuf message name or unique short name
        :returns: The dynamically generated protobuf message class
        :rtype: Type[Message]
        :raises KeyError: If the name is missing or ambiguous in this bundle
        """
        return self.message(full_name)

    def __contains__(self, full_name: str) -> bool:
        """
        Return whether a name resolves uniquely, without generating a class.

        :param str full_name: Fully qualified protobuf message name or short name
        :returns: True if the name resolves uniquely; False if missing or ambiguous
        :rtype: bool
        """
        return self.has_message(full_name)

    def __iter__(self) -> Iterator[str]:
        """
        Iterate over all fully qualified names without generating classes.

        :returns: Iterator over fully qualified message names, including nested messages
        :rtype: Iterator[str]
        """
        return iter(self._full_names)

    def __len__(self) -> int:
        """
        Return the number of message definitions, including nested messages.

        :returns: Total definition count, regardless of how many classes are cached
        :rtype: int
        """
        return len(self._full_names)

    def __repr__(self) -> str:
        """
        Describe the bundle path and total message-definition count.

        :returns: A readable representation of this bundle
        :rtype: str
        """
        return (
            f"{self.__class__.__name__}("
            f"path={str(self._descriptor_path)!r}, "
            f"messages={len(self)})"
        )
