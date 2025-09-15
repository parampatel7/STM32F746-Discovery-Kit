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
: '
number=1
#until will run till the condition becomes True, while will run till the condition is true
#while [ $number -le 10 ]
until [ $number -ge 10 ]
do
	echo "$number"
	number=$(( number+1 ))
done
'

: '
#for i in 1 2 3 4 5
#for i in {0..20..2}
for (( i=0; i<5; i++ ))
do
	echo $i
done
'	

: '
for (( i=0; i<10; i++ ))
do
	if [ $i -gt 5 ]
	then
		break
	fi
	echo $i
done

for (( i=0; i<10; i++ ))
do
	if [ $i -eq 7 ] || [ $i -eq 3 ]
	then
		continue
	fi
	echo $i
done
'

: '
#in terminal: ./helloscript.sh a b b
#echo $1 $2 $3

#in terminal: ./helloscript.sh aaaaa bbbbb ccccc
echo $0 $1 $2 $3
'

: '
#in terminal: ./helloscript.sh bb cc dd
args=("$@")
#echo ${args[0]} ${args[1]} ${args[2]}
echo $@ #prints every element
echo $# #prints length of an array
'

: '
while read line
do 
	echo "$line"
done < "${1:-/dev/stdin}"

#in terminal: ./helloscript.sh C\ learnings.txt 
#above statement will read the whole file
#in terminal: ./helloscript.sh, pressing enter will let the terminal be the file for the script, Type a line and press enter, it will read and print the same line below
'

: '
ls -al 1>file1.txt 2>file2.txt
#run the script, it will create two files
#1> will create a standard output file with all details of every file 
#2> will create standart error file

ls +al 1>file1.txt 2>file2.txt
#this will create error as ls +al is error command, visit file 2

ls -al 1>file.txt
#will assume standard output file
ls +al 1>file.txt
#will not create error file

ls -al >file.txt 2>&1
#this will create both output file and error file in one file
ls +all >file.txt 2>&1

ls -al >&file.txt  #similar to ls -al >file.txt 2>&1
'


: '
message="Param Patel"
export message
./secondscript.sh
'

#in terminal: nl -ba helloscript.sh | less
#This will print whole script file with line numbers

: '
echo "Enter first String: "
read st1

echo "Enter second String: "
read st2

if [ "$st1" == "$st2" ]
then
	echo "String Matched"
else
	echo "String dont Match"
fi

# String length comparison
len1=${#st1}
len2=${#st2}

if [ "$len1" -lt "$len2" ]
then
    echo "Length of \"$st1\" is smaller than \"$st2\" "
elif [ "$len1" -gt "$len2" ]
then
    echo "Length of \"$st1\" is larger than \"$st2\" "
else 
    echo "Strings are equal in length"
fi

# String concatenation
C=$st1$st2
echo "$C"

#upper case/lower case
echo ${st1^}
echo ${st2^^}

echo ${st1^p} #capitalizes first letter
'


: '
n1=4
n2=7
echo $(( n1 + n2 ))
echo $(( n1 - n2 ))
echo $(( n1 * n2 ))
echo $(( n1 / n2 ))
echo $(( n1 % n2 ))

echo $(expr $n1 + $n2)
echo $(expr $n1 - $n2)
echo $(expr $n1 \* $n2)
echo $(expr $n1 / $n2)
echo $(expr $n1 % $n2)
'

: '
echo "Enter Hex number:"
read hex

echo -n "Decimal value of $hex is : "
echo "obase=10; ibase=16; $hex" | bc
'

#learn declare commands in bash scripting

: '
#arrays
car=('bmw' 'rolls' 'pagani' 'buggati')
echo "${car[@]}"
echo "${car[0]}"
echo "${!car[@]}"
echo "${#car[@]}"

unset car[0]
echo "${car[@]}"
echo "${car[0]}"
echo "${!car[@]}"
echo "${#car[@]}"

car[2]='mercedeze'
echo "${car[@]}"
echo "${car[0]}"
echo "${!car[@]}"
echo "${#car[@]}"
'

: '
#functions
function functionName()
{
	echo "This is new function "
}
functionName

function functionPrint()
{
	echo $1 $2 $3 $4 $5 
}
functionPrint This is data to be printed and I will be missed

function functionCheck()
{
	returningvalue="Using function right now"
	echo "$returningvalue"
}
functionCheck

function functioncheck1()
{
	returningvalue="I use Linux"
}
returningvalue="I use windows"
echo $returningvalue
functioncheck1
echo $returningvalue
#This is because variable is updated after calling the function
'

: '
#Files
#This will create a directory
mkdir -p Directory2

echo "Enter directory name to check"
read direct
if [ -d "$direct" ]
# -d for directory
# -f for file
then
	echo "$direct exists"
else
	echo "$direct does not exist"
fi

#create a file
echo "Enter Filename to create"
read fileName
touch $fileName

#append data in a file
echo "Enter the file name which you want to append:"
read filenamez
if [[ -f "$filenamez" ]]
then
	echo "Enter the text You want to append"
	read fileText
	echo "$fileText" >> $filenamez
	#echo "$fileText" >> $filenamez : This will replace the text
else
	echo "$filenamez doesnt exist"
fi


#read a file
echo "Enter the file name which you want to read:"
read file1
if [[ -f "$file1" ]]
then
	while IFS= read -r line
	do
		echo "$line"
	done < $file1
else
	echo "$file1 doesnt exist"
fi


#delete a file

echo "Enter the file name which you want to append:"
read file2
if [[ -f "$file2" ]]
then
	rm $file2
	echo "File has been deleted"
else
	echo "$file2 doesnt exist"
fi
'


#sending email via script
#https://www.youtube.com/watch?v=e7BufAVwDiM
# 2:00:00

: '
url="https://proof.ovh.net/files/1Mb.dat"
#curl ${url} -O  #-O to use the same name as the file which is being downloaded
#curl ${url} -o NewFileDownload #-o is for options
#curl ${url} > NewFileDownload #both same
curl -I ${url} 
'

: '
select car in BMW Defender Pagani Rolls
do
	#echo "You have selected $car"
	case $car in 
	BMW)
		echo "BMW Selected" ;;
	Defender)
		echo "Defender Selected" ;;
	Pagani)
		echo "Pagani Selected" ;;
	Rolls)
		echo "Rolls Selected" ;;
	*)
		echo "Please select between 1..5" ;;
	esac
done
'

: '
echo "Press any key to continue"
while [ true ]
do
	read -t 3 -n 1
if [ $? = 0 ]
then
	echo "You have terminated script"
	exit;
else
	echo "Waiting for you to press the key"
fi
done 
'

: '
mkdir -p temp/NewFolder
inotifywait -m temp/NewFolder
#open the folder and see changes in terminal
#create file, print something by:-
#touch file1.txt, echo "Param"
'

: '
echo "Enter File name to search text from: "
read fileName

if [[ -f "$fileName" ]]
then
	echo "Enter text to search"
	read grepvar
	grep -i -n -c "$grepvar" "$fileName"
	grep -i -v "$grepvar" "$fileName"
#-i removes case sensitivity, -n lists the line number, -c counts the number of times the word exists in file, -v shows all the lines that are without that word
else
	echo " File doesnt exist: $fileName"
fi
#to learn more about grep: 
#Terminal: man grep
'

echo "Enter File name to print from awk: " C learnings.txt
read fileName

if [[ -f "$fileName" ]]
then
	#awk '{print}' "$fileName"
	awk '/code/ {print}' "$fileName" #prints the line containing code
else
	echo " File doesnt exist: $fileName"
fi


