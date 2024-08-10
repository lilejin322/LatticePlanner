from typing import List
from common.Vec2d import Vec2d
from common.AABox2d import AABox2d
from dataclasses import dataclass

@dataclass
class AABoxKDTreeParams:
    """
    Parameters for configuring an Axis-Aligned Bounding Box KD-Tree.
    """

    max_depth: int = -1
    """The maximum depth of the kdtree."""
    max_leaf_size: int = -1
    """The maximum number of items in one leaf node."""
    max_leaf_dimension: float = -1.0
    """The maximum dimension size of leaf node."""

class AABoxKDTree2d:
    """
    The class of KD-tree of Aligned Axis Bounding Box(AABox).
    """

    def __init__(self, objects: List[ObjectType], params: AABoxKDTreeParams):
        """
        Constructor which takes a vector of objects and parameters.
        
        :param List[ObjectType] objects: the objects to be stored in the KD-tree
        :param AABoxKDTreeParams params: Parameters to build the KD-tree.
        """

        self._root: AABoxKDTree2dNode = None
        if len(objects) > 0:
            self._root.reset(AABoxKDTree2dNode(objects, params, 0))

    def GetNearestObject(self, point: Vec2d) -> ObjectType:
        """
        Get the nearest object to a target point.

        :param Vec2d point: The target point. Search it's nearest object.
        :returns: The nearest object to the target point.
        :rtype: ObjectType
        """

        if self._root is None:
            return None
        return self._root.GetNearestObject(point)

    def GetObjects(self, point: Vec2d, distance: float) -> List[ObjectType]:
        """
        Get objects within a distance to a point.

        :param Vec2d point: The center point of the range to search objects.
        :param float distance: The radius of the range to search objects.
        :returns: All objects within the specified distance to the specified point.
        :rtype: List[ObjectType]
        """

        if self._root is None:
            return []
        return self._root.GetObjects(point, distance)

    def GetBoundingBox(self) -> AABox2d:
        """
        Get the axis-aligned bounding box of the objects.

        :returns: The axis-aligned bounding box of the objects.
        :rtype: AABox2d
        """

        if self._root is None:
            return AABox2d()
        return self._root.GetBoundingBox()
