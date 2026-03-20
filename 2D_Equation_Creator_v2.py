## Importing ABAQUS Data and Python modules ##

from abaqus import *
from abaqusConstants import *
import __main__
import math
import section
import regionToolset
import displayGroupMdbToolset as dgm
import part
import material
import assembly
import step
import interaction
import load
import mesh
import job
import sketch
import visualization
import xyPlot
import displayGroupOdbToolset as dgo
import connectorBehavior
import time
import os
import sys
import ctypes
import multiprocessing


# --- Configuration ---
model_name = 'Model-4'
instance_name = 'Matrix-1' 
a = mdb.models[model_name].rootAssembly
nodes = a.instances[instance_name].nodes
tol = 1e-6

# 1. Determine Geometric Bounds & Center
x_coords = [n.coordinates[0] for n in nodes]
y_coords = [n.coordinates[1] for n in nodes]
x_min, x_max = min(x_coords), max(x_coords)
y_min, y_max = min(y_coords), max(y_coords)

mid_x = (x_max + x_min) / 2.0
mid_y = (y_max + y_min) / 2.0

# 2. Create Reference Points in Assembly
rp_n_feat = a.ReferencePoint(point=(x_max + 0.1, y_max + 0.1, 0.0))
rp_s_feat = a.ReferencePoint(point=(x_min - 0.1, y_min + 0.1, 0.0))

# Create Assembly-Level Sets for RPs
a.Set(name='RP-Normal', referencePoints=(a.referencePoints[rp_n_feat.id],))
a.Set(name='RP-Shear', referencePoints=(a.referencePoints[rp_s_feat.id],))

# 3. Find and Fix Central Node
# Find the node closest to the mathematical center
center_node = min(nodes, key=lambda n: (n.coordinates[0]-mid_x)**2 + (n.coordinates[1]-mid_y)**2)
a.Set(name='Center_Node', nodes=a.instances[instance_name].nodes.sequenceFromLabels([center_node.label]))

# Apply BC to fix the center node in space (Prevents Rigid Body Motion)
mdb.models[model_name].DisplacementBC(name='Fix_Center', createStepName='Initial', 
    region=a.sets['Center_Node'], u1=SET, u2=SET, ur3=UNSET, amplitude=UNSET, 
    distributionType=UNIFORM, fieldName='', localCsys=None)

# 4. Helper Function for Assembly Node Sets
def make_assembly_node_set(set_name, node_obj):
    # This specifically creates the set at the Assembly level using instance nodes
    a.Set(name=set_name, nodes=a.instances[instance_name].nodes.sequenceFromLabels([node_obj.label]))

# 5. Identify Corners and Internal Edges
node_BL = [n for n in nodes if abs(n.coordinates[0]-x_min)<tol and abs(n.coordinates[1]-y_min)<tol][0]
node_BR = [n for n in nodes if abs(n.coordinates[0]-x_max)<tol and abs(n.coordinates[1]-y_min)<tol][0]
node_TL = [n for n in nodes if abs(n.coordinates[0]-x_min)<tol and abs(n.coordinates[1]-y_max)<tol][0]
node_TR = [n for n in nodes if abs(n.coordinates[0]-x_max)<tol and abs(n.coordinates[1]-y_max)<tol][0]

make_assembly_node_set('Corner_BL', node_BL)
make_assembly_node_set('Corner_BR', node_BR)
make_assembly_node_set('Corner_TL', node_TL)
make_assembly_node_set('Corner_TR', node_TR)

left_nodes = [n for n in nodes if abs(n.coordinates[0]-x_min)<tol and y_min+tol < n.coordinates[1] < y_max-tol]
right_nodes = [n for n in nodes if abs(n.coordinates[0]-x_max)<tol and y_min+tol < n.coordinates[1] < y_max-tol]
bottom_nodes = [n for n in nodes if abs(n.coordinates[1]-y_min)<tol and x_min+tol < n.coordinates[0] < x_max-tol]
top_nodes = [n for n in nodes if abs(n.coordinates[1]-y_max)<tol and x_min+tol < n.coordinates[0] < x_max-tol]

# 6. Equations for Left-Right Edges
for n_l in left_nodes:
    n_r = [n for n in right_nodes if abs(n.coordinates[1] - n_l.coordinates[1]) < tol][0]
    
    set_l, set_r = 'Set-L-%d' % n_l.label, 'Set-R-%d' % n_r.label
    make_assembly_node_set(set_l, n_l)
    make_assembly_node_set(set_r, n_r)
    
    # X-Displacement mapped to RP-Normal (DOF 1)
    mdb.models[model_name].Equation(name='Eq-LR-%d-dof1'%n_l.label, terms=(
        (1.0, set_r, 1), (-1.0, set_l, 1), (-1.0, 'RP-Normal', 1)))
        
    # Y-Displacement mapped to RP-Shear (DOF 2)
    mdb.models[model_name].Equation(name='Eq-LR-%d-dof2'%n_l.label, terms=(
        (1.0, set_r, 2), (-1.0, set_l, 2), (-1.0, 'RP-Shear', 1)))

# 7. Equations for Bottom-Top Edges
for n_b in bottom_nodes:
    n_t = [n for n in top_nodes if abs(n.coordinates[0] - n_b.coordinates[0]) < tol][0]
    
    set_b, set_t = 'Set-B-%d' % n_b.label, 'Set-T-%d' % n_t.label
    make_assembly_node_set(set_b, n_b)
    make_assembly_node_set(set_t, n_t)
    
    # X-Displacement mapped to RP-Shear (DOF 1)
    mdb.models[model_name].Equation(name='Eq-BT-%d-dof1'%n_b.label, terms=(
        (1.0, set_t, 1), (-1.0, set_b, 1), (-1.0, 'RP-Shear', 2)))
        
    # Y-Displacement mapped to RP-Normal (DOF 2)
    mdb.models[model_name].Equation(name='Eq-BT-%d-dof2'%n_b.label, terms=(
        (1.0, set_t, 2), (-1.0, set_b, 2), (-1.0, 'RP-Normal', 2)))

# 8. Corner Equations mapped to the RPs
# BR relative to BL
mdb.models[model_name].Equation(name='Eq-Corner-BR-dof1', terms=((1.0, 'Corner_BR', 1), (-1.0, 'Corner_BL', 1), (-1.0, 'RP-Normal', 1)))
mdb.models[model_name].Equation(name='Eq-Corner-BR-dof2', terms=((1.0, 'Corner_BR', 2), (-1.0, 'Corner_BL', 2), (-1.0, 'RP-Shear', 1)))

# TL relative to BL
mdb.models[model_name].Equation(name='Eq-Corner-TL-dof1', terms=((1.0, 'Corner_TL', 1), (-1.0, 'Corner_BL', 1), (-1.0, 'RP-Shear', 2)))
mdb.models[model_name].Equation(name='Eq-Corner-TL-dof2', terms=((1.0, 'Corner_TL', 2), (-1.0, 'Corner_BL', 2), (-1.0, 'RP-Normal', 2)))

# TR relative to TL
mdb.models[model_name].Equation(name='Eq-Corner-TR-dof1', terms=((1.0, 'Corner_TR', 1), (-1.0, 'Corner_TL', 1), (-1.0, 'RP-Normal', 1)))
mdb.models[model_name].Equation(name='Eq-Corner-TR-dof2', terms=((1.0, 'Corner_TR', 2), (-1.0, 'Corner_TL', 2), (-1.0, 'RP-Shear', 1)))