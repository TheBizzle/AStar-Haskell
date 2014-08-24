module AStar.AStarLike(runAStar) where

  import Control.Arrow
  import Control.Monad.State

  import Data.Array.IArray
  import Data.Traversable as Traversable
  import Data.Heap as Heap
  import Data.Maybe
  import Data.Set as Set

  import PathFindingCore.PathingMap
  import PathFindingCore.Status
  import PathFindingCore.PathingMap.Coordinate
  import PathFindingCore.PathingMap.Interpreter

  import AStar.Heuristic

  a |> f = f a

  type CoordQueue = Heap MinPolicy (PriorityBundle LocationData)
  type AStarState = State AStarStepData

  runAStar :: PathingMapString -> (Status, AStarStepData)
  runAStar = initialize >>> run
    where
      decide (Continue, sd) = run sd
      decide x              = x
      run                   = (runState performAStar) >>> decide

  performAStar :: AStarState Status
  performAStar = do
    SD _ gsdArr _ queue _ visiteds <- get
    let popped      = getFreshLoc (\x -> member x visiteds) queue
    let updateSD    = (\c h a g s -> s { queue = h, gridSDArr = a // [(c, Just g)] })
    let g           = (\((PBundle _ loc@(Loc coord gsd)), heap) -> modify' (updateSD coord heap gsdArr gsd) >> (doIteration loc))
    let maybeResult = Traversable.mapM g popped
    fmap (fromMaybe Failure) maybeResult

  doIteration :: LocationData -> AStarState Status
  doIteration freshLoc = do
    SD (ImmSD _ maxIters _ _) _ _ _ iters _ <- get
    if (iters < maxIters)
      then doIterationH freshLoc
      else return Failure

  doIterationH :: LocationData -> AStarState Status
  doIterationH freshLoc@(Loc freshCoord _) = do
    sd@(SD (ImmSD _ _ _ goal) _ _ _ iters visiteds) <- doIterationHH
    let newSet   = Set.insert freshCoord visiteds
    let newIters = iters + 1
    put $ sd { visiteds = newSet, iters = newIters, locPair = freshLoc }
    let status = if goal == freshCoord then Success else Continue
    gets (\_ -> status)

  doIterationHH :: AStarState AStarStepData
  doIterationHH = do
    SD (ImmSD _ _ grid _) _ (Loc loc _) _ _ visiteds <- get
    let f = (\neighbor -> when (not $ member neighbor visiteds) $ modify' $ modifyStepState neighbor)
    let newState = mapM_ f $ neighborsOf loc grid
    mapState (\(_, s) -> (s, s)) newState

  modifyStepState :: Coordinate -> AStarStepData -> AStarStepData
  modifyStepState neighbor sd@(SD (ImmSD hValueOf _ _ _) gridSD (Loc _ (GridSD lB lC)) queue _ _) = freshSD
    where
      newCost = lC + 1
      freshSD = gridSD |> ((! neighbor) >>> (fmap $ cost >>> (newCost <)) >>> process)
        where
          process (Just False) = sd --Data exists and the new cost isn't any better
          process _            = sd { queue = newQueue }
            where
              hValue   = hValueOf neighbor
              loc      = Loc neighbor $ GridSD (Crumb neighbor lB) newCost
              pcor     = PBundle (newCost + hValue) loc
              newQueue = Heap.insert pcor queue

  maxItersBy :: Int -> Int -> Double -> Int
  maxItersBy rows cols branchingFactor = rows |> ((*cols) >>> fromIntegral >>> (*branchingFactor) >>> floor)

  getFreshLoc :: (Coordinate -> Bool) -> CoordQueue -> Maybe (PriorityBundle LocationData, CoordQueue)
  getFreshLoc checkIsFamiliar queue = view trimmedQueue
    where
      trimmedQueue = Heap.dropWhile (item >>> lcoord >>> checkIsFamiliar) queue

  initialize :: PathingMapString -> AStarStepData
  initialize pmStr = SD immData gridSD (Loc start startData) queue 0 Set.empty
    where
      (PathingMapData start goal grid) = fromMapString pmStr
      (_, Coord maxX maxY) = bounds grid
      heuristicFunc = manhattanDistance goal
      maxIters      = maxItersBy maxX maxY 1.0
      queue         = Heap.empty
      startData     = GridSD (Source start) 0
      gridSD        = (fmap (\_ -> Nothing) grid) // [(start, Just $ startData)]
      immData       = ImmSD heuristicFunc maxIters grid goal

  data AStarStepData
    = SD {
      unSD      :: ImmutableStepData,
      gridSDArr :: Array Coordinate (Maybe GridStepData),
      locPair   :: LocationData,
      queue     :: CoordQueue,
      iters     :: Int,
      visiteds  :: Set Coordinate -- We could do away with this and some of the cost-checking logic, I think, if we generalize this for only unidirectional A* with all edges having the same weight --JAB (8/23/14)
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
