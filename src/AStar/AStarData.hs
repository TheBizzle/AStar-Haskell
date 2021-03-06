module AStar.AStarData where

import Data.Array.IArray(Array)
import Data.Heap(Heap, MinPolicy)

import PathFindingCore.PathingMap.Coordinate(Breadcrumb, Coordinate)
import PathFindingCore.PathingMap.Interpreter(PathingGrid)

type CoordQueue = Heap MinPolicy (PriorityBundle LocationData)
type SDGrid     = Array Coordinate (Maybe GridStepData)

data AStarStepData
  = SD {
    _unSD       :: ImmutableStepData,
    _gridSDArr  :: SDGrid,
    _locPair    :: LocationData,
    _queue      :: CoordQueue,
    _iters      :: Int,
    _visiteds   :: Set Coordinate -- We could do away with this and some of the cost-checking logic, I think, if we generalize this for only unidirectional A* with all edges having the same weight --JAB (8/23/14)
  }

data ImmutableStepData
  = ImmSD {
    hValueOf :: Coordinate -> Double,
    maxIters :: Int,
    grid     :: PathingGrid,
    goal     :: Coordinate
  }

data GridStepData
  = GridSD {
    breadcrumb :: Breadcrumb,
    cost       :: Double
  } deriving (Eq, Show)

data LocationData
  = Loc {
    lcoord :: Coordinate,
    lsd    :: GridStepData
  } deriving (Eq, Show)

data PriorityBundle a
  = PBundle {
    priority :: Double,
    item     :: a
  } deriving (Eq, Show)

instance Eq x => Ord (PriorityBundle x) where
  compare (PBundle p1 _) (PBundle p2 _) = compare p1 p2
