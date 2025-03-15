#This imports all the required libraries

# -*- coding: mbcs -*-
from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
from functools import reduce


#This enforces the software to save everything in coordinate system

session.journalOptions.setValues(replayGeometry=COORDINATE, recoverGeometry=COORDINATE)

radius=10
fiber_centers = [
    (44, 38),
    (77, 80),
    (19, 49),
    (45, 65),
    (66, 16),
    (96, 34),
    (-4, 34)
]

#This creates the initial rectangular Matrix
mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
mdb.models['Model-1'].sketches['__profile__'].rectangle(point1=(0.0, 0.0), 
    point2=(100.0, 100.0))
mdb.models['Model-1'].Part(dimensionality=TWO_D_PLANAR, name='Composite', type=
    DEFORMABLE_BODY)
mdb.models['Model-1'].parts['Composite'].BaseShell(sketch=
    mdb.models['Model-1'].sketches['__profile__'])
del mdb.models['Model-1'].sketches['__profile__']

for i in range(len(fiber_centers)):
    mdb.models['Model-1'].ConstrainedSketch(gridSpacing=1.41, name='__profile__', 
        sheetSize=200, transform=
        mdb.models['Model-1'].parts['Composite'].MakeSketchTransform(
        sketchPlane=mdb.models['Model-1'].parts['Composite'].faces.findAt((
        0.1, 0.1, 0.0), (0.0, 0.0, 1.0)), sketchPlaneSide=SIDE1, 
        sketchOrientation=RIGHT, origin=(0.0, 0.0, 0.0)))
        
    mdb.models['Model-1'].parts['Composite'].projectReferencesOntoSketch(filter=
        COPLANAR_EDGES, sketch=mdb.models['Model-1'].sketches['__profile__'])
#    mdb.models['Model-1'].sketches['__profile__'].sketchOptions.setValues(
#        gridOrigin=(-10.0, -10.0))
    mdb.models['Model-1'].sketches['__profile__'].CircleByCenterPerimeter(center=(
        fiber_centers[i][0], fiber_centers[i][1]), point1=((fiber_centers[i][0]+radius), fiber_centers[i][1]))
    mdb.models['Model-1'].parts['Composite'].PartitionFaceBySketch(faces=
        mdb.models['Model-1'].parts['Composite'].faces.findAt(((0.1, 0.1, 
        0.0), )), sketch=mdb.models['Model-1'].sketches['__profile__'])
    del mdb.models['Model-1'].sketches['__profile__']
 

#This creates the materials named 
#This create the materials for EGlass as fiber and The Matrix
mdb.models['Model-1'].Material(name='Inclusion')
mdb.models['Model-1'].materials['Inclusion'].Elastic(table=((73000000000.0, 0.3), 
    ))
mdb.models['Model-1'].Material(name='Matrix')
mdb.models['Model-1'].materials['Matrix'].Elastic(table=((1308000000.0, 
    0.4), ))
mdb.models['Model-1'].materials['Matrix'].Plastic(scaleStress=None, 
    table=((40000000.0, 0.0), ))
    
    

#This creates the sections
mdb.models['Model-1'].HomogeneousSolidSection(material='Inclusion', name=
    'Section_Inclusion', thickness=None)
mdb.models['Model-1'].HomogeneousSolidSection(material='Matrix', name=
    'Section_Matrix', thickness=None)
     
    
#This creates the instance    
mdb.models['Model-1'].rootAssembly.DatumCsysByDefault(CARTESIAN)
mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='Composite_Instance', 
    part=mdb.models['Model-1'].parts['Composite'])
    
    
#This creates the mesh
mdb.models['Model-1'].parts['Composite'].seedPart(deviationFactor=0.1, 
    minSizeFactor=0.1, size=30.0)
mdb.models['Model-1'].parts['Composite'].setMeshControls(
    elemShape=TRI, 
    regions=mdb.models['Model-1'].parts['Composite'].faces[:])
   
mdb.models['Model-1'].parts['Composite'].generateMesh()
 
#................................................................Nodal Set Creation...................................................................#  

part =  mdb.models['Model-1'].parts['Composite']
# Get all nodes
nodes = part.nodes

#This creates the set 'XFront'
# Find the right-side edge nodes
tol = 1e-6  # Tolerance to handle floating-point precision
x_max = max(node.coordinates[0] for node in nodes)  # Maximum x-coordinate (right edge)

# Filter right edge nodes
right_edge_nodes = [node for node in nodes if abs(node.coordinates[0] - x_max) < tol]

# Find corner nodes (top-right and bottom-right)
y_values = [node.coordinates[1] for node in right_edge_nodes]
y_max = max(y_values)
y_min = min(y_values)

corner_nodes_xfront = [node for node in right_edge_nodes if abs(node.coordinates[1] - y_max) < tol or abs(node.coordinates[1] - y_min) < tol]

# Remove corner nodes to get internal nodes
internal_right_nodes = [node for node in right_edge_nodes if node not in corner_nodes_xfront]

# Extract node labels from internal_right_nodes
node_labels = [node.label for node in internal_right_nodes]

# Use sequenceFromLabels() to get a valid sequence of nodes
node_sequence = mdb.models['Model-1'].parts['Composite'].nodes.sequenceFromLabels(node_labels)

# Create the node set using the sequence
mdb.models['Model-1'].parts['Composite'].Set(name='XFront', nodes=node_sequence)


# Extract node labels into a list
xfront_node_labels = [node.label for node in internal_right_nodes]

# Print the list of node labels
print("Node labels in XFront:", xfront_node_labels)
#This creates a set for each interal node on right edge
for node_label in xfront_node_labels:
    node_label = int(node_label)
    # Create the node set using the sequence
    node= part.nodes[(node_label-1):(node_label)]
    part.Set(name='N%d' %node_label, nodes=node)




#This creates the set 'XBack'
# Find the right-side edge nodes
tol = 1e-6  # Tolerance to handle floating-point precision
x_min = min(node.coordinates[0] for node in nodes)  # Maximum x-coordinate (right edge)

# Filter internal nodes (excluding top-right and bottom-right corners)
left_edge_nodes = [node for node in nodes if abs(node.coordinates[0] - x_min) < tol]

# Find corner nodes (top-right and bottom-right)
y_values = [node.coordinates[1] for node in left_edge_nodes]
y_max = max(y_values)
y_min = min(y_values)

corner_nodes_xback = [node for node in left_edge_nodes if abs(node.coordinates[1] - y_max) < tol or abs(node.coordinates[1] - y_min) < tol]

# Remove corner nodes to get internal nodes
internal_left_nodes = [node for node in left_edge_nodes if node not in corner_nodes_xback]

# Extract node labels from internal_right_nodes
node_labels = [node.label for node in internal_left_nodes]

# Use sequenceFromLabels() to get a valid sequence of nodes
node_sequence = mdb.models['Model-1'].parts['Composite'].nodes.sequenceFromLabels(node_labels)

# Create the node set using the sequence(Needs Editing)
mdb.models['Model-1'].parts['Composite'].Set(name='XBack', nodes=node_sequence)


# Extract node labels into a list
xBack_node_labels = [node.label for node in internal_left_nodes]

# Print the list of node labels(Needs Editing)
print("Node labels in XBack:", xBack_node_labels)
#This creates a set for each interal node on right edge
for node_label in xBack_node_labels:
    node_label = int(node_label)
    # Create the node set using the sequence
    node= part.nodes[(node_label-1):(node_label)]
    part.Set(name='N%d' %node_label, nodes=node)
    

#This creates the set 'YBottom'
# Find the right-side edge nodes
tol = 1e-6  # Tolerance to handle floating-point precision
y_min = min(node.coordinates[1] for node in nodes)  # Maximum x-coordinate (right edge)

# Filter internal nodes (excluding top-right and bottom-right corners)
bottom_edge_nodes = [node for node in nodes if abs(node.coordinates[1] - y_min) < tol]

# Find corner nodes (top-right and bottom-right)
x_values = [node.coordinates[0] for node in bottom_edge_nodes]
x_max = max(x_values)
x_min = min(x_values)

corner_nodes_ybottom = [node for node in bottom_edge_nodes if abs(node.coordinates[0] - x_max) < tol or abs(node.coordinates[0] - x_min) < tol]

# Remove corner nodes to get internal nodes
internal_bottom_nodes = [node for node in bottom_edge_nodes if node not in corner_nodes_ybottom]

# Extract node labels from internal_right_nodes
node_labels = [node.label for node in internal_bottom_nodes]

# Use sequenceFromLabels() to get a valid sequence of nodes
node_sequence = part.nodes.sequenceFromLabels(node_labels)

# Create the node set using the sequence(Needs Editing)
part.Set(name='YBottom', nodes=node_sequence)


# Extract node labels into a list
yBottom_node_labels = [node.label for node in internal_bottom_nodes]

# Print the list of node labels(Needs Editing)
print("Node labels in YBottom:", yBottom_node_labels)
#This creates a set for each interal node on right edge
for node_label in yBottom_node_labels:
    node_label = int(node_label)
    # Create the node set using the sequence
    node= part.nodes[(node_label-1):(node_label)]
    part.Set(name='N%d' %node_label, nodes=node)  
    
    
#This creates the set 'YTop'
# Find the right-side edge nodes
tol = 1e-6  # Tolerance to handle floating-point precision
y_max = max(node.coordinates[1] for node in nodes)  # Maximum x-coordinate (right edge)

# Filter internal nodes (excluding top-right and bottom-right corners)
top_edge_nodes = [node for node in nodes if abs(node.coordinates[1] - y_max) < tol]

# Find corner nodes (top-right and bottom-right)
x_values = [node.coordinates[0] for node in top_edge_nodes]
x_max = max(x_values)
x_min = min(x_values)
                                                
corner_nodes_ytop = [node for node in top_edge_nodes if abs(node.coordinates[0] - x_max) < tol or abs(node.coordinates[0] - x_min) < tol]

# Remove corner nodes to get internal nodes
internal_top_nodes = [node for node in top_edge_nodes if node not in corner_nodes_ytop]
# print(internal_top_nodes)
# print(internal_top_nodes[0].label)
# for node in internal_top_nodes:
    # print(node)
# Extract node labels from internal_right_nodes
yTop_node_labels = [node.label for node in internal_top_nodes]

# Use sequenceFromLabels() to get a valid sequence of nodes
node_sequence = part.nodes.sequenceFromLabels(yTop_node_labels)

# Create the node set using the sequence(Needs Editing)
part.Set(name='YTop', nodes=node_sequence)

# Print the list of node labels(Needs Editing)
print("Node labels in YTop:", yTop_node_labels)
#This creates a set for each interal node on right edge
for node_label in yTop_node_labels:
    node_label = int(node_label)
    # Create the node set using the sequence
    node= part.nodes[(node_label-1):(node_label)]
    part.Set(name='N%d' %node_label, nodes=node)  
    
    
#This creates the corner nodes set
corner_nodes = corner_nodes_ytop  + corner_nodes_ybottom +  corner_nodes_xback  +  corner_nodes_xfront 
corner_node_labels = list(set([node.label for node in corner_nodes]))

# Print the result
print("Node labels in Corners:", corner_node_labels)
   
# Use sequenceFromLabels() to get a valid sequence of nodes
node_sequence = part.nodes.sequenceFromLabels(corner_node_labels)
# Create the node set using the sequence(Needs Editing)
part.Set(name='Corner_Nodes', nodes=node_sequence)

 # Print the result
print("Node labels in Corners:", corner_node_labels)
#This creates a set for each interal node on right edge
for node_label in corner_node_labels:
    node_label = int(node_label)
    # Create the node set using the sequence
    node= part.nodes[(node_label-1):(node_label)]
    part.Set(name='N%d' %node_label, nodes=node) 
    



x_list = []
#This creates the node pairing of x edge
for node1 in internal_left_nodes:
    distance=1e4
    for node2 in internal_right_nodes:
        distance1= ((node1.coordinates[0]-node2.coordinates[0])**2 + (node1.coordinates[1]-node2.coordinates[1])**2+(node1.coordinates[2]-node2.coordinates[2])**2)**0.5
        
        #print("Distance between node %d and node %d is %d" % (node1.label, node2.label, distance1))

        if distance>distance1:
            distance=distance1
            node=node2
    x_list.append((node1.label, node.label))
    
print(x_list)   

#This creates the node pairing of y edge   
y_list = []
#This creates the node pairing of x edge
for node1 in internal_bottom_nodes:
    distance=1e4
    for node2 in internal_top_nodes:
        distance1= ((node1.coordinates[0]-node2.coordinates[0])**2 + (node1.coordinates[1]-node2.coordinates[1])**2+(node1.coordinates[2]-node2.coordinates[2])**2)**0.5       
        #print("Distance between node %d and node %d is %d" % (node1.label, node2.label, distance1))
        if distance>distance1:
            distance=distance1
            node=node2
    y_list.append((node1.label, node.label))
print(y_list)

#This creates the equation constraint between left and right edge in x direction
for i in range(len(x_list)):
    mdb.models['Model-1'].Equation(name='Eqn-%d%dx' % (x_list[i][1], x_list[i][0]), terms=((1.0, 
    'Composite_Instance.N%d' %x_list[i][1] , 1), (-1.0, 'Composite_Instance.N%d' % x_list[i][0], 1), (-1.0, 
    'Composite_Instance.N7', 1), (1.0, 'Composite_Instance.N6' , 1)))
    
#This creates the equation constraint between top and bottom edge in x direction
for k in range(len(y_list)):
    mdb.models['Model-1'].Equation(name='Eqn-%d%dx' % (y_list[k][1], y_list[k][0]), terms=((1.0, 
    'Composite_Instance.N%d' %y_list[k][1] , 1), (-1.0, 'Composite_Instance.N%d' % y_list[k][0], 1), (-1.0, 
    'Composite_Instance.N11', 1), (1.0, 'Composite_Instance.N6' , 1)))
   
   
#This creates the equation constraint between left and right edge in y direction
for i in range(len(x_list)):
    mdb.models['Model-1'].Equation(name='Eqn-%d%dy' % (x_list[i][1], x_list[i][0]), terms=((1.0, 
    'Composite_Instance.N%d' %x_list[i][1] , 2), (-1.0, 'Composite_Instance.N%d' % x_list[i][0], 2), (-1.0, 
    'Composite_Instance.N7', 2), (1.0, 'Composite_Instance.N6' , 2)))
    
#This creates the equation constraint between top and bottom edge in x direction
for k in range(len(y_list)):
    mdb.models['Model-1'].Equation(name='Eqn-%d%dy' % (y_list[k][1], y_list[k][0]), terms=((1.0, 
    'Composite_Instance.N%d' %y_list[k][1] , 2), (-1.0, 'Composite_Instance.N%d' % y_list[k][0], 2), (-1.0, 
    'Composite_Instance.N11', 2), (1.0, 'Composite_Instance.N6' , 2)))
    
#This creates the constraint equation between the remaining corner node
mdb.models['Model-1'].Equation(name='Eqn-1011x', terms=((1.0, 
    'Composite_Instance.N10', 1), (-1.0, 'Composite_Instance.N11', 1), (-1.0, 
    'Composite_Instance.N7', 1), (1.0, 'Composite_Instance.N6' , 1)))
mdb.models['Model-1'].Equation(name='Eqn-1011y', terms=((1.0, 
    'Composite_Instance.N10', 2), (-1.0, 'Composite_Instance.N11', 2), (-1.0, 
    'Composite_Instance.N7', 2), (1.0, 'Composite_Instance.N6' , 2)))
    