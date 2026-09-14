# gnuradio blocks

This directory contains collections of blocks by topic.

Structure of a block, entries in square brackets are optional:

- <library_name>
  - include/gnuradio-4.0/<library_name>/
    - one or more headers which each can contain one or more block definitions
  - test
    - qa\_<blockname> - tests for a block
    - CMakeLists.txt
  - README.md - a short description of the block library
  - [src] - containing optional samples
  - [assets] - additional block documentation

A module that was renamed out of `gr::<library_name>` carries a deprecated compatibility import beside its
headers, so that the old spelling of a block keeps resolving; a new module ships none, because it has no earlier
spelling to be compatible with.
