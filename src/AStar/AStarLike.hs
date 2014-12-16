{-# LANGUAGE MultiWayIf #-}
module AStar.AStarLike(runAStar) where

import Prelude hiding (iterate)

import Control.Arrow((>>>))
import Control.Monad.State(get, modify, runState, State, when)

import Data.Array.IArray((!), (//), bounds)
import Data.Heap(view)
import Data.Maybe(isJust)
import Data.Set(member, Set)

import qualified Data.Heap        as Heap
import qualified Data.Set         as Set
import qualified Data.Traversable as Traversable

import PathFindingCore.PathingMap(neighborsOf)
import PathFindingCore.PathingMap.Coordinate(Breadcrumb(Crumb, Source), Coordinate(Coord))
import PathFindingCore.PathingMap.Interpreter(fromMapString, PathingMapData(PathingMapData), PathingMapString)
import PathFindingCore.Status(RunResult(FailedRun, SuccessfulRun), Status(Continue, Failure, Success))

import AStar.AStarData(AStarStepData(gridSDArr, iters, locPair, queue, SD, visiteds), CoordQueue, GridStepData(cost, GridSD), ImmutableStepData(ImmSD), LocationData(lcoord, Loc), PriorityBundle(item, PBundle), SDGrid)
import AStar.Heuristic(manhattanDistance)

a |> f = f a

type AStarState = State AStarStepData
type PLocData   = PriorityBundle LocationData

runAStar :: PathingMapString -> (RunResult, AStarStepData)
runAStar = initialize >>> run
  where
    run                   = (runState iterate) >>> decide
    decide (Success, x)   = (SuccessfulRun, x)
    decide (Failure, x)   = (FailedRun, x)
    decide (Continue, sd) = run sd

iterate :: AStarState Status
iterate =
  do
    isPrimed <- primeState
    SD (ImmSD _ maxIters grid goal) _ (Loc freshCoord _) _ iters visiteds <- get
    if | goal == freshCoord                -> return Success
       | not isPrimed || iters >= maxIters -> return Failure
       | otherwise -> do
           mapM_ (updateStateIfFresh visiteds) $ neighborsOf freshCoord grid
           let visiteds' = Set.insert freshCoord visiteds
           modify (\sd -> sd { visiteds = visiteds', iters = iters + 1 })
           return Continue
             where
               updateStateIfFresh :: Set Coordinate -> Coordinate -> AStarState ()
               updateStateIfFresh seen neighbor =
                 when (not $ member neighbor seen) $ modify (\sd -> sd { queue = enqueueNeighbor neighbor sd })

enqueueNeighbor :: Coordinate -> AStarStepData -> CoordQueue
enqueueNeighbor neighbor (SD (ImmSD hValueOf _ _ _) gridSD (Loc _ (GridSD lB lC)) queue _ _) = updatedQueue
  where
    newCost      = lC + 1
    updatedQueue = gridSD |> ((! neighbor) >>> (fmap $ cost >>> (newCost <)) >>> genQueue)
      where
        genQueue (Just False) = queue --Data exists and the new cost isn't any better
        genQueue _            = Heap.insert pcor queue
          where
            hValue = hValueOf neighbor
            loc    = Loc neighbor $ GridSD (Crumb neighbor lB) newCost
            pcor   = PBundle (newCost + hValue) loc

primeState :: AStarState Bool
primeState =
  do
    SD _ grid _ queue _ visiteds <- get
    let updateSD = (updateStepData grid) >>> modify
    queue |> ((popWhile $ flip member visiteds) >>> (Traversable.mapM updateSD) >>> (fmap isJust))
  where
    popWhile :: (Coordinate -> Bool) -> CoordQueue -> Maybe (PLocData, CoordQueue)
    popWhile checkIsFamiliar = (Heap.dropWhile $ item >>> lcoord >>> checkIsFamiliar) >>> view

    updateStepData :: SDGrid -> (PLocData, CoordQueue) -> AStarStepData -> AStarStepData
    updateStepData grid (PBundle _ loc@(Loc coord gridSD), heap) sd =
      sd { queue = heap, gridSDArr = updatedGrid, locPair = loc }
        where
          updatedGrid = grid // [(coord, Just gridSD)]

initialize :: PathingMapString -> AStarStepData
initialize pmStr = SD immData gridSD (Loc start startData) queue 0 Set.empty
  where
    (PathingMapData start goal grid) = fromMapString pmStr
    (_, Coord maxX maxY) = bounds grid
    heuristicFunc = manhattanDistance goal
    maxIters      = maxItersBy (maxX + 1) (maxY + 1) 1.0
    startData     = GridSD (Source start) 0
    queue         = Heap.fromList [PBundle 0 $ Loc start startData]
    gridSD        = (fmap (const Nothing) grid) // [(start, Just startData)]
    immData       = ImmSD heuristicFunc maxIters grid goal

maxItersBy :: Int -> Int -> Double -> Int
maxItersBy rows cols branchingFactor = rows |> ((*cols) >>> fromIntegral >>> (*branchingFactor) >>> floor)
