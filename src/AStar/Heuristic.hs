module AStar.Heuristic(euclideanDistance, HeuristicFunc, manhattanDistance) where

import Control.Arrow((>>>))

import PathFindingCore.PathingMap.Coordinate(Coordinate(Coord))

a |> f = f a

type HeuristicFunc = Coordinate -> Coordinate -> Double

euclideanDistance :: HeuristicFunc
euclideanDistance (Coord x1 y1) (Coord x2 y2) = expDist |> (fromIntegral >>> sqrt)
  where
    squared = (^(2 :: Int))
    expDist = (x1 - x2) |> squared + (y1 - y2) |> squared

manhattanDistance :: HeuristicFunc
manhattanDistance (Coord x1 y1) (Coord x2 y2) = fromIntegral dist
  where
    dist = (abs $ x1 - x2) + (abs $ y1 - y2)
