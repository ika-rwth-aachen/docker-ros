#!/bin/sh -l

echo "Hello $1"
echo "REPO $GITHUB_REPOSITORY"
time=$(date)
echo "time=$time" >> $GITHUB_OUTPUT