import numpy as np

def get_coordinates(coords_path):
    with open(coords_path, "r") as f:
        f.readline() # Skip the first line
        data = []
        for line in f:
            data.append(line.strip().split(","))

    return np.array(data, dtype=np.uint8)

def get_instructions(data):
    """
    TriggerR: Ir Adelante
    TriggerL: Ir Atras
    Derecha: Girar Derecha
    Izquierda: Girar Izquierda
    """
    instructions = ["TriggerR"]

    y1, x1 = data[0]
    y2, x2 = data[1]

    if x2 == x1 and y2 > y1:
        state = "y"
    elif x2 == x1 and y2 < y1:
        state = "-y"
    elif x2 > x1 and y2 == y1:
        state = "x"
    elif x2 < x1 and y2 == y1:
        state = "-x"

    for i in range(2, data.shape[0] - 1):
        y1, x1 = data[i]
        y2, x2 = data[i + 1]

        if state == "y":
            if x2 < x1:
                v = "Derecha"
                state = "-x"
            elif x2 > x1:
                v = "Izquierda"
                state = "x"
            else:
                v = "TriggerR"
        elif state == "-y":
            if x2 < x1:
                v = "Izquierda"
                state = "-x"
            elif x2 > x1:
                v = "Derecha"
                state = "x"
            else:
                v = "TriggerR"
        elif state == "x":
            if y2 < y1:
                v = "Izquierda"
                state = "-y"
            elif y2 > y1:
                v = "Derecha"
                state = "y"
            else:
                v = "TriggerR"
        elif state == "-x":
            if y2 < y1:
                v = "Derecha"
                state = "-y"
            elif y2 > y1:
                v = "Izquierda"
                state = "y"
            else:
                v = "TriggerR"

        # Si el último movimiento guardado no es TriggerR y el movimiento a guardar tampoco, entonces se elimina el último movimiento guardado y se guarda el nuevo movimiento
        if instructions[-1] != "TriggerR" and v != "TriggerR":
            instructions.pop()
            instructions.append(v)
        else:
            instructions.append(v)

    return instructions

def save_instructions(instructions, instructions_path):
    with open(instructions_path, "w") as f:
        count = 0
        for instruction in instructions:
            if instruction == "TriggerR":
                count += 1
            else:
                f.write(("TriggerR\n")*(int(count//(3.5))))
                f.write(("QUIETO\n")*15)
                count = 0
                if instruction == "Derecha":
                    instruction = "Izquierda"
                elif instruction == "Izquierda":
                    instruction = "Derecha"
                f.write((instruction + "\n")*20)

def to_instructions(coords_path, instructions_path):
    print(coords_path, instructions_path)
    data = get_coordinates(coords_path)
    instructions = get_instructions(data)
    save_instructions(instructions, instructions_path)

if __name__ == "__main__":
    to_instructions("path.txt", "instructions.txt")
