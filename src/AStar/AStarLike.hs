module AStar.AStarLike(runAStar) where

import Prelude hiding (iterate)

import Control.Arrow((>>>))
import Control.Monad.State(get, mapState, modify', put, runState, State, when)

import Data.Array.IArray((!), (//), bounds)
import Data.Heap(view)
import Data.Maybe(fromMaybe)
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
    decide (Continue, sd) = run sd
    decide (Failure, x)   = (FailedRun, x)
    decide (Success, x)   = (SuccessfulRun, x)
    run                   = (runState performAStar) >>> decide

performAStar :: AStarState Status
performAStar =
  do
    SD _ gsdArr _ queue _ visiteds <- get
    let poppedDataAndQueue = getFreshLoc (flip member visiteds) queue
    let state              = iterateForStatus gsdArr poppedDataAndQueue
    fmap (fromMaybe Failure) state
  where
    updateState :: SDGrid -> (PLocData, CoordQueue) -> AStarState ()
    updateState gsdArr (PBundle _ loc, heap) = modify' $ updateStepData heap gsdArr loc

    updateStepData :: CoordQueue -> SDGrid -> LocationData -> AStarStepData -> AStarStepData
    updateStepData heap grid loc@(Loc coord gridSD) sd = sd { queue = heap, gridSDArr = updatedGrid, locPair = loc }
      where
        updatedGrid = grid // [(coord, Just gridSD)]

    iterateForStatus :: SDGrid -> Maybe (PLocData, CoordQueue) -> AStarState (Maybe Status)
    iterateForStatus grid = Traversable.mapM $ (updateState grid) >>> (>> iterate)

iterate :: AStarState Status
iterate =
  do
    SD (ImmSD _ maxIters _ goal) _ (Loc freshCoord _) _ iters visiteds <- get
    if (iters < maxIters)
      then do
        sd <- iterateOnNeighbors
        let newSet = Set.insert freshCoord visiteds
        put $ sd { visiteds = newSet, iters = iters + 1 }
        return $ if goal == freshCoord
                   then Success
                   else Continue
      else return Failure

iterateOnNeighbors :: AStarState AStarStepData
iterateOnNeighbors =
  do
    SD (ImmSD _ _ grid _) _ (Loc freshCoord _) _ _ visiteds <- get
    let newState = mapM_ (updateStateIfFresh visiteds) $ neighborsOf freshCoord grid
    mapState (\(_, s) -> (s, s)) newState
  where
    updateStateIfFresh :: Set Coordinate -> Coordinate -> AStarState ()
    updateStateIfFresh seen neighbor = when (not $ member neighbor seen) $ modify' (\sd -> sd { queue = enqueueNeighbor neighbor sd })

enqueueNeighbor :: Coordinate -> AStarStepData -> CoordQueue
enqueueNeighbor neighbor (SD (ImmSD hValueOf _ _ _) gridSD (Loc _ (GridSD lB lC)) queue _ _) = updatedQueue
  where
    newCost = lC + 1
    updatedQueue = gridSD |> ((! neighbor) >>> (fmap $ cost >>> (newCost <)) >>> genQueue)
      where
        genQueue (Just False) = queue --Data exists and the new cost isn't any better
        genQueue _            = Heap.insert pcor queue
          where
            hValue   = hValueOf neighbor
            loc      = Loc neighbor $ GridSD (Crumb neighbor lB) newCost
            pcor     = PBundle (newCost + hValue) loc

maxItersBy :: Int -> Int -> Double -> Int
maxItersBy rows cols branchingFactor = rows |> ((*cols) >>> fromIntegral >>> (*branchingFactor) >>> floor)

getFreshLoc :: (Coordinate -> Bool) -> CoordQueue -> Maybe (PLocData, CoordQueue)
getFreshLoc checkIsFamiliar queue = view trimmedQueue
  where
    trimmedQueue = Heap.dropWhile (item >>> lcoord >>> checkIsFamiliar) queue

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
