module AStar.AStarLike(getFreshLoc, maxItersBy) where

  import Control.Arrow

  import Data.Heap as Heap

  import PathFindingCore.PathingMap.Coordinate

  a |> f = f a

  type CoordQueue = Heap MinPolicy PriorityCoordinate

  maxItersBy :: Int -> Int -> Double -> Int
  maxItersBy rows cols branchingFactor = rows |> ((*cols) >>> fromIntegral >>> (*branchingFactor) >>> floor)

  getFreshLoc :: (Coordinate -> Bool) -> CoordQueue -> Maybe (PriorityCoordinate, CoordQueue)
  getFreshLoc checkIsFamiliar queue = view trimmedQueue
    where
      trimmedQueue = Heap.dropWhile (coord >>> checkIsFamiliar) queue
