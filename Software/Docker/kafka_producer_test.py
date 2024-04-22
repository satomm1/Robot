from kafka import KafkaProducer
import time, json

def serializer(message):
        return json.dumps(message).encode("utf-8")

if __name__ == "__main__":
    print("Hello Kafka")

    producer = KafkaProducer(
        bootstrap_servers=["192.168.50.2:29094"],
        value_serializer=serializer
    )

    x = False
    while True:
        if x:
            producer.send("quickstart-events", "a")
            time.sleep(1)
            x = False
        else:
            producer.send("quickstart-events", "b")
            time.sleep(1)
            x = True
