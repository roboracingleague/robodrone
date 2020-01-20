#!/bin/sh
### to push your code on your remote drone

REMOTE_USER=dlinano

[[ "$1" == "" ]] && echo "Error, you must specify the IP of your remote drone" && exit 1
IPDEST=$1

ssh $REMOTE_USER@$IPDEST "rm -fr ~/robodrone/*" && scp -r -p ./* dlinano@$IPDEST:~/robodrone/.
