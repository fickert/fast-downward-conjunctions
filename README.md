This is a fork of [Fast Downward](http://www.fast-downward.org/) extended with an implementation of the hCFF heuristic and online refinement of explicit conjunctions.
This implementation has been used for experiments in the following literature:

* J. Hoffmann and M. Fickert: [Explicit Conjunctions w/o Compilation: Computing hFF(PiC) in Polynomial Time](http://fai.cs.uni-saarland.de/hoffmann/papers/icaps15b.pdf) (ICAPS'15)
* M. Fickert, J. Hoffmann, and M. Steinmetz: [Combining the Delete Relaxation with Critical-Path Heuristics: A Direct Characterization](https://jair.org/index.php/jair/article/view/11027) (JAIR'16)
* M. Fickert and J. Hoffmann: [Complete Local Search: Boosting Hill-Climbing through Online Relaxation Refinement](http://fai.cs.uni-saarland.de/hoffmann/papers/icaps17a.pdf) (ICAPS'17)
* M. Fickert and J. Hoffmann: [Ranking Conjunctions for Partial Delete Relaxation Heuristics in Planning](http://fai.cs.uni-saarland.de/hoffmann/papers/socs17b.pdf) (SOCS'17)
* M. Fickert: [Making Hill-Climbing Great Again through Online Relaxation Refinement and Novelty Pruning](http://fai.cs.uni-saarland.de/fickert/papers/socs18.pdf) (SOCS'18)
* M. Fickert: [A Novel Lookahead Strategy for Delete Relaxation Heuristics in Greedy Best-First Search](http://fai.cs.uni-saarland.de/fickert/papers/icaps20.pdf) (ICAPS'20)
* M. Fickert and J. Hoffmann: [Online Relaxation Refinement for Satisficing Planning: On Partial Delete Relaxation, Complete Hill-Climbing, and Novelty Pruning](https://jair.org/index.php/jair/article/view/13153) (JAIR'22)

It is also the code base of OLCFF and parts of Saarplan; both planners competed in the [International Planning Competition 2018](https://ipc2018-classical.bitbucket.io/).

The hCFF heuristic and online-refinement search engines are implemented in `src/search/conjunctions`.
The best-performing configurations of RHC, RHC-SC, and GBFS-SCL from the JAIR'22 paper have corresponding aliases (see `driver/aliases.py`), so they can be run as follows:

```
./fast-downward.py --alias RHC task.pddl
./fast-downward.py --alias RHC-SC task.pddl
./fast-downward.py --alias GBFS-SCL task.pddl
```

The `jair22-data.tar.xz` file contains the raw data from the JAIR'22 experiments (in the Downward Lab JSON format).

The code also contains a copy of the h2 preprocessor (Alcazar and Torralba, ICAPS'15), which can be enabled by prepending the `--transform-task=preprocess` driver option.

## Fast Downward

Fast Downward is a domain-independent planning system.

For documentation and contact information see http://www.fast-downward.org/.

The following directories are not part of Fast Downward as covered by this
license:

* ./src/search/ext

For the rest, the following license applies:

```
Fast Downward is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Fast Downward is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
```
