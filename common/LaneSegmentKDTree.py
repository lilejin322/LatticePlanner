from typing import List, Any, Tuple
from common.Vec2d import Vec2d
from common.AABox2d import AABox2d
from dataclasses import dataclass
from enum import Enum
from Polygon2d import kMathEpsilon

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

class Partition(Enum):

    PARTITION_X = 1
    PARTITION_Y = 2

class AABoxKDTree2dNode:
    """
    The class of KD-tree node of axis-aligned bounding box.
    """

    def __init__(self, objects: List[Any], params: AABoxKDTreeParams, depth: int):
        """
        Constructor which takes a vector of objects, parameters and depth of the node.

        :param List[Any] objects: Objects to build the KD-tree node.
        :param AABoxKDTreeParams params: Parameters to build the KD-tree.
        :param int depth: Depth of the KD-tree node.
        """

        self._depth = depth
        assert len(objects) > 0, "objects is empty!"
        self._num_objects: int = 0
        self._objects_sorted_by_min = []
        self._objects_sorted_by_max = []
        self._objects_sorted_by_min_bound = []
        self._objects_sorted_by_max_bound = []

        self._partition: Partition = Partition.PARTITION_X
        self._partition_position: float = 0.0

        # Boundary
        self._min_x = float('inf')
        self._min_y = float('inf')
        self._max_x = float('-inf')
        self._max_y = float('-inf')

        self._left_subnode: AABoxKDTree2dNode = None
        self._right_subnode: AABoxKDTree2dNode = None

        self.ComputeBoundary(objects)
        self.ComputePartition()

        if self.SplitToSubNodes(objects, params):
            left_subnode_objects, right_subnode_objects = self.PartitionObjects(objects)

            # Split to sub-nodes.
            if left_subnode_objects:
                self._left_subnode = AABoxKDTree2dNode(left_subnode_objects, params, depth + 1)
            if right_subnode_objects:
                self._right_subnode = AABoxKDTree2dNode(right_subnode_objects, params, depth + 1)
        else:
            self.InitObjects(objects)

    def GetNearestObject(self, point: Vec2d) -> Any:
        """
        Get the nearest object to a target point by the KD-tree rooted at this node.

        :param Vec2d point: The target point. Search it's nearest object.
        :returns: The nearest object to the target point.
        :rtype: Any
        """

        nearest_object = None
        min_distance_sqr = float("inf")
        _, nearest_object = self.GetNearestObjectInternal(point, min_distance_sqr)
        return nearest_object

    def GetObjects(self, point: Vec2d, distance: float) -> List[Any]:
        """
        Get objects within a distance to a point by the KD-tree rooted at this node.

        :param Vec2d point: The center point of the range to search objects.
        :param float distance: The radius of the range to search objects.
        :returns: All objects within the specified distance to the specified point.
        :rtype: List[Any]
        """

        result_objects: List[Any] = self.GetObjectsInternal(point, distance, distance ** 2)
        return result_objects

    def GetBoundingBox(self) -> AABox2d:
        """
        Get the axis-aligned bounding box of the objects.

        :returns: The axis-aligned bounding box of the objects.
        """

        return AABox2d((self._min_x, self._min_y), (self._max_x, self._max_y))

    def InitObjects(self, objects: List[Any]) -> None:
        """
        Initialize the objects in the KD-tree node.

        :param List[Any] objects: The objects to be stored in the KD-tree node.
        """

        self._num_objects = len(objects)
        self._objects_sorted_by_min = objects[:]
        self._objects_sorted_by_max = objects[:]

        self._objects_sorted_by_min.sort(key=lambda obj: obj.aabox().min_x if self._partition == Partition.PARTITION_X else obj.aabox().min_y)
        self._objects_sorted_by_max.sort(key=lambda obj: obj.aabox().max_x if self._partition == Partition.PARTITION_X else obj.aabox().max_y, reverse=True)

        self._objects_sorted_by_min_bound = []
        for obj in self._objects_sorted_by_min:
            min_bound = obj.aabox().min_x if self._partition == Partition.PARTITION_X else obj.aabox().min_y
            self._objects_sorted_by_min_bound.append(min_bound)
        
        self._objects_sorted_by_max_bound = []
        for obj in self._objects_sorted_by_max:
            max_bound = obj.aabox().max_x if self._partition == Partition.PARTITION_X else obj.aabox().max_y
            self._objects_sorted_by_max_bound.append(max_bound)

    def SplitToSubNodes(self, objects: List[Any], params: AABoxKDTreeParams) -> bool:
        """
        Split the objects to sub-nodes.

        :param List[Any] objects: The objects to be split.
        :param AABoxKDTreeParams params: Parameters to build the KD-tree.
        :returns: True if the objects should be split to sub-nodes, False otherwise.
        :rtype: bool
        """

        if params.max_depth >= 0 and self._depth >= params.max_depth:
            return False
        if len(objects) <= max(1, params.max_leaf_size):
            return False
        if params.max_leaf_dimension >= 0.0 and max(self._max_x - self._min_x, self._max_y - self._min_y) <= params.max_leaf_dimension:
            return False
        return True

    def LowerDistanceSquareToPoint(self, point: Vec2d) -> float:
        """
        Compute the square of the lower distance to a point.

        :param Vec2d point: The target point.
        :returns: The square of the lower distance to the target point.
        :rtype: float
        """

        dx: float = 0.0
        if point.x < self._min_x:
            dx = self._min_x - point.x
        elif point.x > self._max_x:
            dx = point.x - self._max_x
        dy: float = 0.0
        if point.y < self._min_y:
            dy = self._min_y - point.y
        elif point.y > self._max_y:
            dy = point.y - self._max_y
        return dx * dx + dy * dy

    def UpperDistanceSquareToPoint(self, point: Vec2d) -> float:
        """
        Compute the square of the upper distance to a point.

        :param Vec2d point: The target point.
        :returns: The square of the upper distance to the target point.
        :rtype: float
        """

        dx: float = (point.x - self._min_x) if point.x > self._mid_x else (point.x - self._max_x)
        dy: float = (point.y - self._min_y) if point.y > self._mid_y else (point.y - self._max_y)
        return dx * dx + dy * dy

    def GetAllObjects(self, result_objects: List[Any]) -> None:
        """
        Get all objects in the KD-tree rooted at this node.

        :param List[Any] result_objects: The list to store all objects.
        """

        result_objects.extend(self._objects_sorted_by_min)
        if self._left_subnode:
            self._left_subnode.GetAllObjects(result_objects)
        if self._right_subnode:
            self._right_subnode.GetAllObjects(result_objects)

    def GetObjectsInternal(self, point: Vec2d, distance: float, distance_sqr: float, result_objects: List[Any]) -> None:
        """
        Get objects within a distance to a point by the KD-tree rooted at this node.

        :param Vec2d point: The center point of the range to search objects.
        :param float distance: The radius of the range to search objects.
        :param float distance_sqr: The square of the distance.
        :param List[Any] result_objects: The list to store all objects.
        """

        if self.LowerDistanceSquareToPoint(point) > distance_sqr:
            return
        if self.UpperDistanceSquareToPoint(point) <= distance_sqr:
            self.GetAllObjects(result_objects)
            return
        pvalue: float = point.x if self._partition == Partition.PARTITION_X else point.y
        if pvalue < self._partition_position:
            limit: float = pvalue + distance
            for i in range(self._num_objects):
                if self._objects_sorted_by_min_bound[i] > limit:
                    break
                obj: Any = self._objects_sorted_by_min[i]
                if obj.DistanceSquareTo(point) <= distance_sqr:
                    result_objects.append(obj)
        else:
            limit: float = pvalue - distance
            for i in range(self._num_objects):
                if self._objects_sorted_by_max_bound[i] < limit:
                    break
                obj: Any = self._objects_sorted_by_max[i]
                if obj.DistanceSquareTo(point) <= distance_sqr:
                    result_objects.append(obj)
        if self._left_subnode is not None:
            self._left_subnode.GetObjectsInternal(point, distance, distance_sqr, result_objects)
        if self._right_subnode is not None:
            self._right_subnode.GetObjectsInternal(point, distance, distance_sqr, result_objects)

    def GetNearestObjectInternal(self, point: Vec2d, min_distance_sqr: float) -> Tuple[float, Any]:
        """
        Get the nearest object to a target point by the KD-tree rooted at this node.

        :param Vec2d point: The target point. Search it's nearest object.
        :param float min_distance_sqr: The old square of the minimum distance.
        :returns: The new square of the distance and the nearest object to the target point.
        :rtype: Tuple[float, Any]
        """

        if self.LowerDistanceSquareToPoint(point) >= min_distance_sqr - kMathEpsilon:
            return min_distance_sqr, None
        pvalue: float = point.x if self._partition == Partition.PARTITION_X else point.y
        search_left_first: bool = pvalue < self._partition_position
        if search_left_first:
            if self._left_subnode is not None:
                min_distance_sqr, nearest_object = self._left_subnode.GetNearestObjectInternal(point, min_distance_sqr)
        else:
            if self._right_subnode is not None:
                min_distance_sqr, nearest_object = self._right_subnode.GetNearestObjectInternal(point, min_distance_sqr)
        if min_distance_sqr <= kMathEpsilon:
            return min_distance_sqr, nearest_object
        
        if search_left_first:
            for i in range(self._num_objects):
                bound: float = self._objects_sorted_by_min_bound[i]
                if bound > pvalue and (bound - pvalue) ** 2 > min_distance_sqr:
                    break
                obj: Any = self._objects_sorted_by_min[i]
                distance_sqr: float = obj.DistanceSquareTo(point)
                if distance_sqr < min_distance_sqr:
                    min_distance_sqr = distance_sqr
                    nearest_object = obj
        else:
            for i in range(self._num_objects):
                bound: float = self._objects_sorted_by_max_bound[i]
                if bound < pvalue and (bound - pvalue) ** 2 > min_distance_sqr:
                    break
                obj: Any = self._objects_sorted_by_max[i]
                distance_sqr: float = obj.DistanceSquareTo(point)
                if distance_sqr < min_distance_sqr:
                    min_distance_sqr = distance_sqr
                    nearest_object = obj
        if min_distance_sqr <= kMathEpsilon:
            return min_distance_sqr, nearest_object
        if search_left_first:
            if self._right_subnode is not None:
                min_distance_sqr, nearest_object = self._right_subnode.GetNearestObjectInternal(point, min_distance_sqr)
        else:
            if self._left_subnode is not None:
                min_distance_sqr, nearest_object = self._left_subnode.GetNearestObjectInternal(point, min_distance_sqr)

        return min_distance_sqr, nearest_object

    def ComputeBoundary(self, objects: List[Any]) -> None:
        """
        Compute the boundary of the objects.

        :param List[Any] objects: The objects to compute the boundary.
        """

        for obj in objects:
            self._min_x = min(self._min_x, obj.aabox().min_x)
            self._max_x = max(self._max_x, obj.aabox().max_x)
            self._min_y = min(self._min_y, obj.aabox().min_y)
            self._max_y = max(self._max_y, obj.aabox().max_y)
        self._mid_x = (self._min_x + self._max_x) / 2.0
        self._mid_y = (self._min_y + self._max_y) / 2.0
        assert self._min_x < float('inf') and self._max_x > float('-inf') and self._min_y < float('inf') and self._max_y > float('-inf'), "the provided object box size is infinity"

    def ComputePartition(self) -> None:
        """
        Compute the partition of the KD-tree node.
        """

        if self._max_x - self._min_x >= self._max_y - self._min_y:
            self._partition = Partition.PARTITION_X
            self._partition_position = (self._min_x + self._max_x) / 2.0
        else:
            self._partition = Partition.PARTITION_Y
            self._partition_position = (self._min_y + self._max_y) / 2.0

    def PartitionObjects(self, objects: List[Any]) -> Tuple[List[Any], List[Any]]:
        """
        Partition the objects to left and right sub-nodes.

        :param List[Any] objects: The objects to be partitioned.
        :returns: The objects in the left sub-node and the objects in the right sub-node.
        :rtype: Tuple[List[Any], List[Any]]
        """

        left_subnode_objects: List[Any] = []
        right_subnode_objects: List[Any] = []
        other_objects: List[Any] = []
        if self._partition == Partition.PARTITION_X:
            for obj in objects:
                if obj.aabox().max_x <= self._partition_position:
                    left_subnode_objects.append(obj)
                elif obj.aabox().min_x >= self._partition_position:
                    right_subnode_objects.append(obj)
                else:
                    other_objects.append(obj)
        else:
            for obj in objects:
                if obj.aabox().max_y <= self._partition_position:
                    left_subnode_objects.append(obj)
                elif obj.aabox().min_y >= self._partition_position:
                    right_subnode_objects.append(obj)
                else:
                    other_objects.append(obj)
        self.InitObjects(other_objects)
        return left_subnode_objects, right_subnode_objects

class AABoxKDTree2d:
    """
    The class of KD-tree of Aligned Axis Bounding Box(AABox).
    """

    def __init__(self, objects: List[Any], params: AABoxKDTreeParams):
        """
        Constructor which takes a vector of objects and parameters.
        
        :param List[Any] objects: the objects to be stored in the KD-tree
        :param AABoxKDTreeParams params: Parameters to build the KD-tree.
        """

        self._root: AABoxKDTree2dNode = None
        if len(objects) > 0:
            self._root.reset(AABoxKDTree2dNode(objects, params, 0))

    def GetNearestObject(self, point: Vec2d) -> Any:
        """
        Get the nearest object to a target point.

        :param Vec2d point: The target point. Search it's nearest object.
        :returns: The nearest object to the target point.
        :rtype: Any
        """

        if self._root is None:
            return None
        return self._root.GetNearestObject(point)

    def GetObjects(self, point: Vec2d, distance: float) -> List[Any]:
        """
        Get objects within a distance to a point.

        :param Vec2d point: The center point of the range to search objects.
        :param float distance: The radius of the range to search objects.
        :returns: All objects within the specified distance to the specified point.
        :rtype: List[Any]
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
