#!/bin/sh
echo "Content-Type: text/plain"
echo "Cache-Control: no-cache, must-revalidate"
echo "Expires: Sat, 28 Mar 2020 05:00:00 GMT"
echo

RELAI=/sys/class/leds/tp-link:blue:relay/brightness

case "$QUERY_STRING" in
 state)
  echo `cat $RELAI`
 ;;
 on) 
  echo 1 > $RELAI
  echo `cat $RELAI`
 ;;
 off) 
  echo 0 > $RELAI
  echo `cat $RELAI`
 ;;
esac
                          
                          
