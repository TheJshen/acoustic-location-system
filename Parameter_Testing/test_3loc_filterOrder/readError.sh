#/bin/bash

filename="fOrder_16"
while read -r line
do
    name="$line"
    echo "Name read from file - $name"
done < "$filename"
