from kafka import KafkaConsumer
import json

if __name__ == "__main__":
    print("Hello Kafka!")
    consumer = KafkaConsumer("quickstart-events", bootstrap_servers=">    print("Consumer established")

    for msg in consumer:
        print(msg.value)
