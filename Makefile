all: track_pb2.py td_feed_pb2.py td_feed_pb2_grpc.py

%_pb2.py: %.proto
	protoc -I. --python_out=. $<

%_pb2_grpc.py: %.proto
	python3 -m grpc_tools.protoc -I. --grpc_python_out=. $<
