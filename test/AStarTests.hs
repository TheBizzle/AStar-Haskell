module AStarTests(tests) where

import System.IO.Unsafe(unsafePerformIO)

import Control.Arrow((>>>))
import Control.Lens.Review((#))

import Data.List.NonEmpty(NonEmpty((:|)))
import Data.Validation(_Failure, _Success, Validation)

import qualified Data.List.NonEmpty as NEL
import qualified Data.Map           as Map
import qualified Data.Set           as Set

import Test.Tasty(testGroup, TestTree)
import Test.Tasty.HUnit((@?=), testCase)

import Text.Printf(printf)

import Tester.Dialect(FlagCells, runningTo)
import Tester.RunSettings(cellsToSettings, testNums)
import Tester.Suite(Result, runTests, Suite(Suite), TestResult(TestFailure, TestSuccess))

import PathFindingTest.TestSet(PathingMapTest(..))

import qualified PathFindingTest.TestSet as TestSet

import PathFindingCore.PathingMap(insertPath, PrintablePathingGrid(PPG))
import PathFindingCore.PathingMap.Coordinate(breadcrumbsToList)
import PathFindingCore.PathingMap.Interpreter()
import PathFindingCore.Status(RunResult(FailedRun, SuccessfulRun))

import AStar.AStarData(AStarStepData(SD), GridStepData(GridSD), ImmutableStepData(ImmSD), LocationData(Loc))
import AStar.AStarLike(runAStar)

a |> f = f a

type ResultType = Result String String

tests = testGroup "Test interpreter" $ testBundle $ 1 `runningTo` 39

testBundle :: FlagCells -> [TestTree]
testBundle = (zipFrom genNums genResults) >>> (fmap evalResult)
  where
    genResults = flip runTests defaultTestSuite
    genNums    = cellsToSettings >>> testNums >>> Set.toList

evalResult :: (Int, TestResult) -> TestTree
evalResult (n, r) = testCase (show n) assertion
  where
    res TestSuccess       = True
    res (TestFailure msg) = seq (unsafePerformIO $ putStrLn msg) False
    assertion             = True @?= (res r)

zipFrom :: (x -> [a]) -> (x -> [b]) -> x -> [(a, b)]
zipFrom fas fbs = (diverge fas fbs) >>> (uncurry zip)
  where
    diverge fa fb x = (fa x, fb x)

defaultTestSuite :: Suite PathingMapTest String String
defaultTestSuite = Suite testMap runThatTest failsToStr id
  where
    testMap        = Map.fromList $ Prelude.zip [1..] TestSet.tests
    failsToStr nel = unlines $ NEL.toList nel

runThatTest :: PathingMapTest -> ResultType
runThatTest (PathingMapTest distMaybe pms) = uncurry (analyzeResult distMaybe) $ runAStar pms

analyzeResult :: (Show n, Num n) => Maybe n -> RunResult -> AStarStepData -> ResultType
analyzeResult (Just x) SuccessfulRun sd = _Success # "Pathfinding successful"
analyzeResult Nothing  FailedRun     sd = _Success # "Pathfinding failed successfully"
analyzeResult (Just x) _             sd = failResult (\(badNum, asciiMap) -> printf "Expected to reach goal in %s steps, but it took %s steps.  Here's the map:\n\n%s" (show x) (show badNum) asciiMap) sd
analyzeResult Nothing  _             sd = failResult (\(badNum, asciiMap) -> printf "Did not expect to reach goal, but I somehow did in %s steps.  Here's the map:\n\n%s"       (show badNum) asciiMap) sd

failResult :: ((Double, String) -> String) -> AStarStepData -> Validation (NonEmpty String) String
failResult f sd = _Failure # (sd |> (processData >>> f >>> (:| [])))

processData :: AStarStepData -> (Double, String)
processData (SD (ImmSD _ _ pathingMap _) _ (Loc _ (GridSD breadcrumbs cost)) _ _ _) = (cost, mapString)
  where
    path      = breadcrumbsToList breadcrumbs
    markedMap = insertPath path pathingMap
    mapString = show $ PPG markedMap
