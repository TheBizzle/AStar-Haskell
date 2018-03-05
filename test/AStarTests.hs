module AStarTests(tests) where

import System.IO.Unsafe(unsafePerformIO)

import Control.Lens.Review((#))

import Data.List(zip)
import Data.List.NonEmpty(NonEmpty((:|)))
import Data.Validation(_Failure, _Success, Validation)

import qualified Data.List.NonEmpty as NEL
import qualified Data.Map           as Map
import qualified Data.Set           as Set
import qualified Data.Text.IO       as TIO

import Test.Tasty(testGroup, TestTree)
import Test.Tasty.HUnit((@?=), testCase)

import Text.Printf(printf)

import Tester.Dialect(FlagCells, runningTo)
import Tester.RunSettings(cellsToSettings, testNums)
import Tester.Suite(Result, runTests, Suite(Suite), TestResult(TestFailure, TestSuccess))

import PathFindingTest.TestSet(PathingMapTest(PathingMapTest))

import qualified PathFindingTest.TestSet as TestSet

import PathFindingCore.PathingMap(insertPath, PrintablePathingGrid(PPG))
import PathFindingCore.PathingMap.Coordinate(breadcrumbsToList)
import PathFindingCore.Status(RunResult(FailedRun, SuccessfulRun))

import AStar.AStarData(AStarStepData(SD), GridStepData(GridSD), ImmutableStepData(ImmSD), LocationData(Loc))
import AStar.AStarLike(runAStar)

type ResultType = Result Text Text

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
    res (TestFailure msg) = seq (unsafePerformIO $ TIO.putStrLn msg) False
    assertion             = True @?= (res r)

zipFrom :: (x -> [a]) -> (x -> [b]) -> x -> [(a, b)]
zipFrom fas fbs = (diverge fas fbs) >>> (uncurry zip)
  where
    diverge fa fb x = (fa x, fb x)

defaultTestSuite :: Suite PathingMapTest Text Text
defaultTestSuite = Suite testMap runThatTest failsToStr id
  where
    testMap        = Map.fromList $ zip [1..] TestSet.tests
    failsToStr nel = unlines $ NEL.toList nel

runThatTest :: PathingMapTest -> ResultType
runThatTest (PathingMapTest distMaybe pms) = uncurry (analyzeResult distMaybe) $ runAStar pms

analyzeResult :: (Show n, Num n) => Maybe n -> RunResult -> AStarStepData -> ResultType
analyzeResult (Just x) SuccessfulRun sd = _Success # "Pathfinding successful"
analyzeResult Nothing  FailedRun     sd = _Success # "Pathfinding failed successfully"
analyzeResult (Just x) _             sd = failResult (\(badNum, asciiMap) -> asText $ printf "Expected to reach goal in %s steps, but it took %s steps.  Here's the map:\n\n%s" (show x) (show badNum) asciiMap) sd
analyzeResult Nothing  _             sd = failResult (\(badNum, asciiMap) -> asText $ printf "Did not expect to reach goal, but I somehow did in %s steps.  Here's the map:\n\n%s"       (show badNum) asciiMap) sd

failResult :: ((Double, Text) -> Text) -> AStarStepData -> Validation (NonEmpty Text) Text
failResult f sd = _Failure # (sd |> (processData >>> f >>> (:| [])))

processData :: AStarStepData -> (Double, Text)
processData (SD (ImmSD _ _ pathingMap _) _ (Loc _ (GridSD breadcrumbs cost)) _ _ _) = (cost, mapString)
  where
    path      = breadcrumbsToList breadcrumbs
    markedMap = insertPath path pathingMap
    mapString = showText $ PPG markedMap
