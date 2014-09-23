module Main where

import AStarTests as AStar
import Test.Framework.Runners.Console (defaultMain)

main = defaultMain $ [AStar.tests]
