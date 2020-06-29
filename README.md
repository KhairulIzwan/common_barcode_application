# common_barcode_application

Common barcode (QR-Code, Bar-Code) packages

**Notes**
Require(s):
1. Python barcode decoding library -- Zbar:
	1. sudo apt-get install libzbar0
	2. pip install pyzbar

# node(s)
1. barcode_detection.py
Used to detect the barcode; QR-Code/Bar-Code thus read and publish the value.

2. barcode_identification.py
Used to determine the code (QR/Bar) category; "customer" or "store" once the 
node (1) succesfully publish the barcode value.

3. barcode_record.py
"store" barcode will be assigned with the "available" locker number, recorded,
and email notification; self-generated barcode will be sent to the "customer".

4. barcode_validate.py
"customer" barcode will be validated to see; trigger the designated locker number
based on the node (3) database.

