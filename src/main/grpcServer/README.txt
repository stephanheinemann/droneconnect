If changes are made to the .proto, these files need to be compiled.

To compile open the ubuntu terminal in this folder and run:

python -m grpc_tools.protoc -I../proto/ --python_out=. --grpc_python_out=. droneconnect.proto

More information about grpc in:

https://grpc.io/docs/tutorials/basic/python.html
