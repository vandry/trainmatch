all: track_pb2.py

%_pb2.py: %.proto
	protoc -I. --python_out=. $<
