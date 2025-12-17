# Communication

The material in this folder is responsible for the communication between the different components in the system. The structure mimics the one found in the [incubator case study](https://github.com/clagms/IncubatorDTCourse).

## Contents

- [Communication](#communication)
  - [Contents](#contents)
  - [RabbitMQ](#rabbitmq)
  - [Protocol](#protocol)

## RabbitMQ

RabbitMQ is a message broker that is used to send messages between the different components in the system. Refer to the [0_pre_requisites](/0_pre_requisites/) folder for instructions on how to install RabbitMQ and its basic usage.

## Protocol

In the file [protocol.py](/communication/protocol.py), the protocol for the communication between the different components in the system is defined. The protocol consists of different routing keys that are used to route the messages to the correct component. Moreover, the protocol defines the different types of messages that are sent between the components.
