#!/bin/sh
IPDEST=$1
ssh dlinano@$IPDEST "rm -fr ~/robodrone/*" && scp -r -p ./* dlinano@$IPDEST:~/robodrone/.
