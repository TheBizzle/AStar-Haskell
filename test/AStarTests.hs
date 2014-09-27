module AStarTests(tests) where

import System.IO.Unsafe

import Test.Framework.Providers.API as API
import Test.Framework.Providers.HUnit
import Test.HUnit

import Control.Arrow
import Control.Lens.Review((#))

import Data.List.NonEmpty as NEL hiding (unlines, zip)
import Data.Map           as Map
import Data.Set           as Set
import Data.Validation

import Text.Printf

import Tester.Dialect
import Tester.RunSettings
import Tester.Suite

import PathFindingTest.TestSet as TestSet(PathingMapTest(..))
import qualified PathFindingTest.TestSet as TestSet(tests)

import PathFindingCore.PathingMap
import PathFindingCore.PathingMap.Coordinate
import PathFindingCore.PathingMap.Interpreter
import PathFindingCore.Status(RunResult(..))

import AStar.AStarData
import AStar.AStarLike

a |> f = f a

type ResultType = Result String String

tests = testGroup "Test interpreter" $ testBundle $ 1 `runningTo` 39

testBundle :: FlagCells -> [API.Test]
testBundle = (zipFrom genNums genResults) >>> (fmap evalResult)
  where
    genResults = flip runTests defaultTestSuite
    genNums    = cellsToSettings >>> testNums >>> Set.toList

evalResult :: (Int, TestResult) -> API.Test
evalResult (n, r) = testCase (show n) assertion
  where
    res TestSuccess       = True
    res (TestFailure msg) = seq (unsafePerformIO $ putStrLn msg) False
    assertion             = True @?= (res r)

zipTuple :: ([a], [b]) -> [(a, b)]
zipTuple (as, bs) = zip as bs

diverge :: (x -> a) -> (x -> b) -> x -> (a, b)
diverge fa fb x = (fa x, fb x)

zipFrom :: (x -> [a]) -> (x -> [b]) -> x -> [(a, b)]
zipFrom fas fbs x = zipTuple (diverge fas fbs x)

defaultTestSuite :: Suite PathingMapTest String String
defaultTestSuite = Suite testMap runThatTest failsToStr id
  where
    testMap        = Map.fromList $ Prelude.zip [1..] TestSet.tests
    failsToStr nel = unlines $ NEL.toList nel

runThatTest :: PathingMapTest -> ResultType
runThatTest (PathingMapTest distMaybe pms) = analyzeResult distMaybe result sd
  where
    (result, sd) = runAStar pms

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
    path       = breadcrumbsToList breadcrumbs
    markedMap  = insertPath path pathingMap
    mapString  = show $ PPG markedMap
