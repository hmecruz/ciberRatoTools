#!/bin/bash

challenge="4"
host="localhost"
robname="pClient1"
pos="0"
outfile="planning"

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


source venv/bin/activate

shift $(($OPTIND-1))

case $challenge in
    4)
        # Call the Python agent with the parsed parameters
        cd agent/C4
        python3 main.py \
            --host "$host" \
            --pos "$pos" \
            --robname "$robname" \
            --outfile "$outfile"
        cp $outfile.path ../../
        cp $outfile.map  ../../
        ;;
esac

