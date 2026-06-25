#!/bin/bash
pytest -x -s tests/odDemandsTestSuite.py --pdb
pytest -x -s tests/staticRoutingTestSuite.py --pdb
pytest -x -s tests/outputFilesTestSuite.py --pdb 