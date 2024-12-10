#!/bin/bash

challenge="4"
host="localhost"
robname="pyRemi"
pos="0"

while getopts "c:h:r:p:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR in parameters"
            ;;
    esac
done

shift $(($OPTIND-1))

case $challenge in
    1)
        # how to call agent for challenge 1
        echo "Starting Challenge 1... " ; cd agent/C1 ; python3 main.py
        ;;
    2)
        # how to call agent for challenge 2
        echo "Starting Challenge 2..." ; cd agent/C2 ; python3 main.py
        ;;
    3)
        # how to call agent for challenge 3
        echo "Starting Challenge 3... " ; cd agent/C3 ; python3 main.py
        ;;
    4)
        # how to call agent for challenge 3
        echo "Starting Challenge 4..." ; cd agent/C4 ; python3 main.py
        ;;
esac

