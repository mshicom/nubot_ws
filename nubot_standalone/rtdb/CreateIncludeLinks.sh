# creating symbolic links for directory utils

echo "Creating Include Links..."
echo

for i in rtdb comm example;
do 
	(cd include/nubot/rtdb ; find ../../../$i/ -name \*.h -exec ln -sf '{}' . \; )
done

for i in rtdb comm example;
do 
	(cd include ; find ../$i/ -name \*.h -exec ln -sf '{}' . \; )
done
