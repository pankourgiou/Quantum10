import cirq
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Helper function to plot Bloch vector
def plot_bloch_vector(state_vector, title=''):
    # Only works for 1 qubit
    bloch_vector = cirq.bloch_vector_from_state_vector(state_vector, index=0)

    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111, projection='3d')
    ax.quiver(0, 0, 0, bloch_vector[0], bloch_vector[1], bloch_vector[2], color='blue')
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_title(f"Bloch Sphere\n{title}")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Draw unit sphere
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, color='lightgrey', alpha=0.1)
    plt.show()

# Create a qubit
qubit = cirq.NamedQubit("q0")

# Create a quantum circuit with various gates
circuit = cirq.Circuit()
circuit.append([cirq.X(qubit)**0.5])        # sqrt(X) gate (Hadamard-like)
circuit.append([cirq.Z(qubit)])             # Pauli-Z
circuit.append([cirq.rx(np.pi/4)(qubit)])   # Rotation around X axis
circuit.append([cirq.ry(np.pi/3)(qubit)])   # Rotation around Y axis
circuit.append([cirq.H(qubit)])             # Hadamard

# Display the circuit diagram
print("Circuit:")
print(circuit)

# Simulate the circuit
simulator = cirq.Simulator()
result = simulator.simulate(circuit)

# Output the final state vector
print("\nFinal state vector:")
print(np.round(result.final_state_vector, 3))

# Plot Bloch sphere
plot_bloch_vector(result.final_state_vector, title="Final State on Bloch Sphere")

# Measurement example (optional)
circuit.append(cirq.measure(qubit, key='m'))
measurement_result = simulator.run(circuit, repetitions=10)
print("\nMeasurement samples:")
print(measurement_result)

import cirq
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Helper function to plot Bloch vector
def plot_bloch_vector(state_vector, title=''):
    # Only works for 1 qubit
    bloch_vector = cirq.bloch_vector_from_state_vector(state_vector, index=0)

    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111, projection='3d')
    ax.quiver(0, 0, 0, bloch_vector[0], bloch_vector[1], bloch_vector[2], color='blue')
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_title(f"Bloch Sphere\n{title}")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Draw unit sphere
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, color='lightgrey', alpha=0.1)
    plt.show()

# Create a qubit
qubit = cirq.NamedQubit("q0")

# Create a quantum circuit with various gates
circuit = cirq.Circuit()
circuit.append([cirq.X(qubit)**0.5])        # sqrt(X) gate (Hadamard-like)
circuit.append([cirq.Z(qubit)])             # Pauli-Z
circuit.append([cirq.rx(np.pi/4)(qubit)])   # Rotation around X axis
circuit.append([cirq.ry(np.pi/3)(qubit)])   # Rotation around Y axis
circuit.append([cirq.H(qubit)])             # Hadamard

# Display the circuit diagram
print("Circuit:")
print(circuit)

# Simulate the circuit
simulator = cirq.Simulator()
result = simulator.simulate(circuit)

# Output the final state vector
print("\nFinal state vector:")
print(np.round(result.final_state_vector, 3))

# Plot Bloch sphere
plot_bloch_vector(result.final_state_vector, title="Final State on Bloch Sphere")

# Measurement example (optional)
circuit.append(cirq.measure(qubit, key='m'))
measurement_result = simulator.run(circuit, repetitions=10)
print("\nMeasurement samples:")
print(measurement_result)

import cirq
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Helper function to plot Bloch vector
def plot_bloch_vector(state_vector, title=''):
    # Only works for 1 qubit
    bloch_vector = cirq.bloch_vector_from_state_vector(state_vector, index=0)

    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(111, projection='3d')
    ax.quiver(0, 0, 0, bloch_vector[0], bloch_vector[1], bloch_vector[2], color='blue')
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_title(f"Bloch Sphere\n{title}")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Draw unit sphere
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, color='lightgrey', alpha=0.1)
    plt.show()

# Create a qubit
qubit = cirq.NamedQubit("q0")

# Create a quantum circuit with various gates
circuit = cirq.Circuit()
circuit.append([cirq.X(qubit)**0.5])        # sqrt(X) gate (Hadamard-like)
circuit.append([cirq.Z(qubit)])             # Pauli-Z
circuit.append([cirq.rx(np.pi/4)(qubit)])   # Rotation around X axis
circuit.append([cirq.ry(np.pi/3)(qubit)])   # Rotation around Y axis
circuit.append([cirq.H(qubit)])             # Hadamard

# Display the circuit diagram
print("Circuit:")
print(circuit)

# Simulate the circuit
simulator = cirq.Simulator()
result = simulator.simulate(circuit)

# Output the final state vector
print("\nFinal state vector:")
print(np.round(result.final_state_vector, 3))

# Plot Bloch sphere
plot_bloch_vector(result.final_state_vector, title="Final State on Bloch Sphere")

# Measurement example (optional)
circuit.append(cirq.measure(qubit, key='m'))
measurement_result = simulator.run(circuit, repetitions=10)
print("\nMeasurement samples:")
print(measurement_result)
