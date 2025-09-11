#! /bin/bash

: ' Multiline comment 
#This is for comment'
#cat > file.txt
#cat >> file.txt
: 'cat << creative
this is creative text 
another line
creative'

: '
count=10
#if [ $count -ne 8 ]
#if [ @count -eq 10 ]
#if [ $count -gt 8 ]  #if (( $count > 8 ))
if (( $count < 9))
then 
	echo "True condition"
elif(( $count >= 9 ))
then 
	echo "True in elif block"
else
	echo "False Condition"
fi
'
: '
age=10
#if [ "$age" -gt 18 ] && [ "$age" -lt 40 ]
#if [[ "$age" -gt 18 && "$age" -lt 40 ]]
#if [ "$age" -gt 18 -a "$age" -lt 40 ]
#if [ "$age" -gt 18 -o "$age" -lt 40 ]
if [[ "$age" -gt 18 || "$age" -lt 40 ]]
then
	echo "Age is correct"
else
	echo "Age is not correct"
fi
'
: '
echo "Enter a number between 1 and 3:"
read number

case $number in
    1)
        echo "You chose One.";;
    2)
        echo "You chose Two.";;
    3)
        echo "You chose Three.";;
    *)
        echo "Invalid choice. Please enter 1, 2, or 3.";;
esac
'
 
