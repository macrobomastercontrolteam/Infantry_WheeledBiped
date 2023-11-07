# Infantry_WheeledBiped
2023 Capstone: Two Wheeled-Biped Infantry Robot
## INFANTRY_BIPED Diagram
``` mermaid
---
title: 2023 MacFalcons Infantry_Biped Electrical Assembly
---
%% Naming convention: first prefix is module name
flowchart
    %% subgraph Chassis
        subgraph CV["Jetson Nano (Linux PC)"]
            CV-DcJack["19VDC Power Input"]
            CV-USB-Camera["USB1"]
            CV-CtrltoCv-USBtoTTL["USB-to-TTL to Control"]
        end

        subgraph CtrlBoardC["Type C Control Board (STM32F407IG)"]
            Ctrl-XT30In["Power Input"]
            CtrltoCv-UART2["UART2 for CV (USART1 in code)"]
            CAN1-Chassis["Chassis CAN (CAN1)"]
        end

        subgraph ExtBoard1["ESC Center Board (CAN and XT30 Adaptor)"]
            EB1-XT30["XT30 (7 ports)"]
            EB1-CAN-2Pin["CAN-2Pin (7 ports)"]
            EB1-CAN-4Pin["CAN-4Pin"]
            EB1-XT60["XT60"]
        end

        %% Connections
        Battery["Battery TB47S (24V)"]-->EB1-XT60
        Camera["Camera"] --- CV-USB-Camera["USB1"]
        EB1-XT30-->BuckConvertor["24V-to-19V Buck Convertor"]-->CV-DcJack
        EB1-CAN-2Pin== CAN cable ===Motor-Joint["DM-J8006-2EC Joint Motors\n(Front-right, Front-left, Rear-left, Rear-right)"] & Motor-Drive["RMD-L-9015 Drive Motor\n(Left and Right)"]
        EB1-XT30== XT30 cable ===Motor-Joint & Motor-Drive
        EB1-XT30===Ctrl-XT30In
        CV-CtrltoCv-USBtoTTL["USB-to-TTL to Control"] --- CtrltoCv-UART2["UART2 for CV (USART1 in code)"]
        EB1-CAN-2Pin---CAN1-Chassis
    %% end
```
