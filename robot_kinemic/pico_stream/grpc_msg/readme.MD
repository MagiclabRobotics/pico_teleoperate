更新meta_controller_stream/proto/playcontroller.proto
cd meta_controller_stream/grpc_msg
rm -rf playcontroller_pb2*.py __pycache__/
python3 -m grpc_tools.protoc -I . --python_out=. --grpc_python_out=. ./xrTracking.proto