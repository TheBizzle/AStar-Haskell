{-# LANGUAGE MultiWayIf, TemplateHaskell #-}
module AStar.AStarLike(runAStar) where

import Prelude hiding (iterate)

import Control.Arrow((>>>))
import Control.Lens(makeLenses, set)
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
import PathFindingCore.Status(RunResult(FailedRun, SuccessfulRun))

import AStar.AStarData(AStarStepData(_queue, SD), CoordQueue, GridStepData(cost, GridSD), ImmutableStepData(ImmSD), LocationData(lcoord, Loc), PriorityBundle(item, PBundle), SDGrid)
import AStar.Heuristic(manhattanDistance)

makeLenses ''AStarStepData

a |> f = f a

type AStarState = State AStarStepData
type PLocData   = PriorityBundle LocationData

runAStar :: PathingMapString -> (RunResult, AStarStepData)
runAStar = initialize >>> (runState iterate)

iterate :: AStarState RunResult
iterate =
  do
    isPrimed <- primeState
    SD (ImmSD _ maxIters grid goal) _ (Loc poppedCoord _) _ iterCount visitedCoords <- get
    if | goal == poppedCoord                   -> return SuccessfulRun
       | not isPrimed || iterCount >= maxIters -> return FailedRun
       | otherwise -> do
           mapM_ (updateStateIfFresh visitedCoords) $ neighborsOf poppedCoord grid
           let visitedCoords' = Set.insert poppedCoord visitedCoords
           modify $ (set visiteds visitedCoords') . (set iters $ iterCount + 1)
           iterate
             where
               updateStateIfFresh :: Set Coordinate -> Coordinate -> AStarState ()
               updateStateIfFresh seen neighbor =
                 when (not $ member neighbor seen) $ modify (\sd -> sd { _queue = enqueueNeighbor neighbor sd })

enqueueNeighbor :: Coordinate -> AStarStepData -> CoordQueue
enqueueNeighbor neighbor (SD (ImmSD hValueOf _ _ _) gridSD (Loc _ (GridSD lB lC)) queue _ _) = updatedQueue
  where
    newCost        = lC + 1
    newCostIsLower = cost >>> (newCost <)
    updatedQueue   = gridSD |> ((! neighbor) >>> (fmap newCostIsLower) >>> genQueue)
      where
        genQueue (Just False) = queue -- The coord has already registered a cost that is at least as good
        genQueue _            = Heap.insert pcor queue
          where
            hValue = hValueOf neighbor
            loc    = Loc neighbor $ GridSD (Crumb neighbor lB) newCost
            pcor   = PBundle (newCost + hValue) loc

primeState :: AStarState Bool
primeState =
  do
    SD _ grid _ queue _ visiteds <- get
    let updateSD                 = (updateStepData grid) >>> modify
    let drawUnvisitedCoordMaybe  = popWhile $ flip member visiteds
    let updateSDAndInvert        = Traversable.mapM updateSD
    let determineIfStateIsPrimed = fmap isJust
    queue |> (drawUnvisitedCoordMaybe >>> updateSDAndInvert >>> determineIfStateIsPrimed)
  where
    popWhile :: (Coordinate -> Bool) -> CoordQueue -> Maybe (PLocData, CoordQueue)
    popWhile checkIsFamiliar = (Heap.dropWhile $ item >>> lcoord >>> checkIsFamiliar) >>> view

    updateStepData :: SDGrid -> (PLocData, CoordQueue) -> AStarStepData -> AStarStepData
    updateStepData grid (PBundle _ loc@(Loc coord gridSD), heap) =
      (set queue heap) . (set gridSDArr updatedGrid) . (set locPair loc)
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
