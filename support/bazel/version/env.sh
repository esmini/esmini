#!/bin/bash

echo GIT_COMMIT $(git log -1 --oneline)
echo GIT_TAG $(git describe --tags --abbrev=0)