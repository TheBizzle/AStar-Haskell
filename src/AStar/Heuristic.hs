module AStar.Heuristic(euclideanDistance, manhattanDistance) where

  import Control.Arrow

  import PathFindingCore.PathingMap.Coordinate

  a |> f = f a

  euclideanDistance :: Coordinate -> Coordinate -> Double
  euclideanDistance (Coord x1 y1) (Coord x2 y2) = expDist |> (fromIntegral >>> sqrt)
    where
      expDist = (x1 - x2) ^ 2 + (y1 - y2) ^ 2

  manhattanDistance :: Coordinate -> Coordinate -> Double
  manhattanDistance (Coord x1 y1) (Coord x2 y2) = fromIntegral dist
    where
      dist = (abs $ x1 - x2) + (abs $ y1 - y2)
