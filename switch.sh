#!/bin/sh

if [ $# -ne 2 ]; then
    echo "Usage: git name <email> <name>"
    echo "Current email: $(git config --global user.email)"
    echo "Current name: $(git config --global user.name)"
    exit 0
fi

git config --global user.email "$1"

git config --global user.name "$2"