#!/bin/bash

RED=$(printf '\033[31m')
RESET=$(printf '\033[m')

while getopts "t:" flag; do
    case "${flag}" in
        t)
            runtype=${OPTARG};;
    esac
done
shift $((OPTIND - 1))

build() {
    rm -rf build/
    mkdir build
    cd build
    cmake ..
    make -j4
}

main() {
    echo "${RED}built with <3 by @onlycase_${RESET}"

    if [[ $runtype == "build" ]]; then
        build
    elif [[ $runtype == "run" ]]; then
        echo "${RED}>${RESET} running."
        ./build/EKF data/lidar-radar-synthetic-input.txt
    elif [[ $runtype == "all" ]]; then
        build
        ./EKF ../data/lidar-radar-synthetic-input.txt
    fi
}

main
