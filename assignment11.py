from scipy import *
import numpy as numpy
import matplotlib.pyplot as plt
from random import *
from copy import deepcopy
import os
import time
import sys

def MatrixCreate(x, y):
    v = numpy.zeros([x,y])
    #v2 = v[0]
    return v

def MatrixRandomizePosNeg(v):
    for col in range(len(v)):
        for row in range(len(v[0])):
            p = random()
            x = 0
            if(p < .5):
                x = random() * -1
            else:
                x = random()
                
            v[col][row] = x
    return v

def MatrixRandomZeroOrOne(v):
    for col in range(len(v)):
        for row in range(len(v[0])):
            p = random()
            x = 0
            if(p < .5):
                x = 0
            else:
                x = 1
                
            v[col][row] = x
    return v

def MatrixRandomizeZero(v):
    for col in range(len(v)):
        for row in range(len(v[0])):
            v[col][row] = 0
    return v

def MatrixRandomizeOne(v):
    for col in range(len(v)):
        for row in range(len(v[0])):
            v[col][row] = 1
    return v

def MatrixPerturbZeroOrOne(Parent,Probability):

    p = deepcopy(Parent)

    for row in range(len(p)):
        for col in range(len(p[0])):
            rand = random()
            if(rand<Probability):
                #print "CHANGING!"
                temp = random()
                x = 0
                if(temp < .5):
                    x = 0
                else:
                    x = 1
                p[row][col] = x


    return p

def MatrixPerturb(Parent,Probability):
    p = deepcopy(Parent)
    for row in range(len(Parent)):
        for col in range(len(Parent[0])):
            rand = random()
            if(rand<Probability):
                #print "CHANGING!"
                p[row][col] = uniform(-1,1)
    return p


def Send_Synapse_Weights_ToFile(synapses,weightsFileName):
    #print "SHOW SYNAPSE"
    line = ""
    with open(weightsFileName, 'w') as f:
        for i in range(4):
            for j in range(8):
                f.write("%f\n" % synapses[i][j])
                line = line, synapses[i][j], " "
            #print line
            line = ""
def Send_Mask_ToFile(synapses,weightsFileName):
    #print "SHOWING MASK"
    line = ""
    with open(weightsFileName, 'w') as f:
        for i in range(4):
            for j in range(8):
               f.write("%d\n" % synapses[i][j])
               line = line, synapses[i][j], " "
            #print line
            line = ""
                
def Send_Render_toFile(value,weightsFileName):
    with open(weightsFileName, 'w') as f:
        f.write("%d" % value)

def getSynapseWeights():
    synapses = MatrixCreate(4,8)
    f = open("../../RAGDOLL_DATA/bestParent.txt", "r")        
    #GET RETURNED FITNESS
    for i in range(len(synapses)):
        for j in range(len(synapses[0])):
            synapses[i][j] = f.readline()
    return synapses;

def getSynapseWeightsFromFile(fileLocation):
    synapses = MatrixCreate(4,8)
    f = open(fileLocation, "r")
    #GET RETURNED FITNESS
    for i in range(len(synapses)):
        for j in range(len(synapses[0])):
            synapses[i][j] = f.readline()
    return synapses;

def Fitness3_Get(parentSynapse, parentRecurrent, parentMask):
    #SEND DATA TO FILE FOR BULLET TO READ
    weightsFileName = "..\..\RAGDOLL_DATA\weights.txt";
    fitFileName = "../../RAGDOLL_DATA/fit.txt";
    maskFileName = "..\..\RAGDOLL_DATA/mask.txt";
    recurrentFileName = "..\..\RAGDOLL_DATA/recurrent.txt";
    Send_Mask_ToFile(parentMask, maskFileName)
    Send_Synapse_Weights_ToFile(parentRecurrent,recurrentFileName);
    Send_Synapse_Weights_ToFile(parentSynapse,weightsFileName);
    time.sleep(0.5);

    #DELETE OLD FITS.txt
    if(os.path.isfile(fitFileName)):
        os.remove(fitFileName)
    
    #CALL SIMULATION
    os.system('..\..\App_RagdollDemo_vs2010_debug.exe');
    counter = 0;

    #LOOK FOR FIT FILES
    while(os.path.isfile(fitFileName) == False):
        counter = counter + 1
        time.sleep(0.2)
        
    #GET RETURNED FITNESS
    f = open(fitFileName, "r")
    fitness = f.readline()

    #PAUSE
    time.sleep(0.2)
    
    #DELETE OLD WEIGHTS.txt
    if(os.path.isfile(weightsFileName)):
        os.remove(weightsFileName)
    
    return fitness;


def HillClimberProbe(proGenerations, experimentType):
    #fitness
    bestFitness = -10;

    #synapse
    bestSynapse = MatrixCreate(4,8)
    #recurrent
    bestRecurrent = MatrixCreate(4,8)
    #mask
    bestMask = MatrixCreate(4,8)
    
    for i in range(proGenerations):
        #CREATE RANDOM MASK
        childMask = MatrixCreate(4,8)
        if(experimentType == 0):
            childMask = MatrixRandomizeZero(MatrixCreate(4,8))
        elif(experimentType == 1):
            childMask = MatrixRandomizeOne(MatrixCreate(4,8))
        elif(experimentType == 2):
            childMask = MatrixRandomZeroOrOne(MatrixCreate(4,8))
        #CREATE RANDOM SYNAPSE
        childSynapse = MatrixRandomizePosNeg(MatrixCreate(4,8))
        #CREATE RANDOM RECURRENT
        childRecurrent = MatrixRandomizePosNeg(MatrixCreate(4,8))

        #TURN ON RENDERING SYNAPSE
        #RENDER OR DONT
        if(i % 10 == 0):
            print "----------------RENDERING PROBE----------"
            Send_Render_toFile(1,"../../RAGDOLL_DATA/render.txt");
        else:
            print "----------------NOT RENDERING PROBE----------"
            Send_Render_toFile(0,"../../RAGDOLL_DATA/render.txt");

        #GET FITNESS
        fitness = Fitness3_Get(childSynapse, childRecurrent, childMask)
        print "Probe ", i, ": ", bestFitness, " - ", fitness
        if(double(fitness) > bestFitness):
            bestFitness = double(fitness)
            bestSynapse = childSynapse
            bestRecurrent = childRecurrent
            bestMask = childMask
        

        #SAVE BEST SYNAPSE DATA
        location = "../../RAGDOLL_DATA/RECURRENT_PROBE/BESTsynapse" + str(i) + ".txt"
        Send_Synapse_Weights_ToFile(bestSynapse,location)

        #SAVE BEST MASK
        location = "../../RAGDOLL_DATA/RECURRENT_PROBE/BESTrecurrent" + str(i) + ".txt"
        Send_Synapse_Weights_ToFile(bestRecurrent,location)

        #SAVE BEST MASK
        location = "../../RAGDOLL_DATA/RECURRENT_PROBE/BESTmask" + str(i) + ".txt"
        Send_Mask_ToFile(bestMask,location)
    return bestSynapse, bestRecurrent, bestMask, bestFitness

def HillClimber(generations, incomingSynapse, incomingRecurrent, incomingMask, incomingFitness, experimentType):
    
    #GENERATIONS MATRIX
    generationFitness = MatrixCreate(1, generations)
    generationFitness = generationFitness[0,:]

    #Parent Starts As Incoming
    parentRecurrent = incomingRecurrent
    parentSynapse = incomingSynapse
    parentMask = incomingMask
    parentFitness = incomingFitness


    #RUN THE EVOLUTION!
    for i in range(generations):
        #RENDER OR DONT
        if(i % 10 == 0):
            print "----------------RENDERING----------"
            Send_Render_toFile(1,"../../RAGDOLL_DATA/render.txt");
        else:
            print "----------------NOT RENDERING----------"
            Send_Render_toFile(0,"../../RAGDOLL_DATA/render.txt");
        
        #Create Child Synapse
        childSynapse = MatrixPerturb(parentSynapse, 0.05)

        #Create Child Recurrent
        childRecurrent = MatrixPerturb(parentRecurrent, 0.05);
        
        #Create Child Mask
        if(experimentType == 0):
            childMask = parentMask
        elif(experimentType == 1):
            childMask = parentMask
        elif(experimentType == 2):
            childMask = MatrixPerturbZeroOrOne(parentMask, 0.05)
        
    
        #RUN SIMULATOR WITH NEW CHILD
        childFitness = Fitness3_Get(childSynapse, childRecurrent, childMask)

        #CHECK IF CHILD IS BETTER THAN PARENT
        if(double(childFitness) > double(parentFitness)):
            bestSynapse = childSynapse
            parentFitness = childFitness
            parentMask = childMask
            parentRecurrent = childRecurrent;
        
        #Add fitness to generation matrix
        generationFitness[i] = parentFitness
        print "Generation: ", i, " | ", parentFitness, " | ", childFitness
             
        #SAVE MASK DATA
        location = "../../RAGDOLL_DATA/RECURRENT/maskData" + str(i) + ".txt"
        Send_Mask_ToFile(parentMask,location);
        
        #SAVE syanpse DATA
        location = "../../RAGDOLL_DATA/RECURRENT/synapseData" + str(i) + ".txt"
        Send_Synapse_Weights_ToFile(parentSynapse,location)

        #SAVE recurrent DATA
        location = "../../RAGDOLL_DATA/RECURRENT/recurrentData" + str(i) + ".txt"
        Send_Synapse_Weights_ToFile(parentRecurrent,location)
        
    return parentSynapse, parentRecurrent, generationFitness

def Main(generations,lineages):
    for index in range(0,lineages):
        parent, generationFitness = HillClimber(generations)

    print "SHOW FINAL GENERATIONS PLOT"
    plot = plt.plot(generationFitness)
    plt.show(plot)

def MainWithProbe(generations,lineages, probes, experimentType):
    for index in range(0,lineages):
        print "Probing Fitness Landscape"
        bestParent, bestRecurrent, bestMask, bestFitness = HillClimberProbe(probes, experimentType)
        print "Starting Evolution"
        parent, bestRecurrent, generationFitness = HillClimber(generations, bestParent, bestRecurrent, bestMask, bestFitness, experimentType)
        
        
    return generationFitness

def MainFromFile(generations,lineages, fileLocation):
    #GET BEST PARENT FROM FILE
    parentSynapse = getSynapseWeightsFromFile(fileLocation)

    print parentSynapse;
    for index in range(0,generations):
        print "Starting Evolution"
        print parentSynapse;
        bestMask = MatrixRandomizeZero(MatrixCreate(4,8))
        bestRecurrent = MatrixRandomizePosNeg(MatrixCreate(4,8))
        bestFitness = 0
        experimentType = 0
        parent, bestRecurrent, generationFitness = HillClimber(generations, parentSynapse, bestRecurrent, bestMask, bestFitness, experimentType);
    print "SHOW FINAL GENERATIONS PLOT"

    return generationFitness    
    

def assign10(generations, lineages, probes):
    print "-------EVOLUTION------"
    fitnessGenerationsWithout = MainWithProbe(generations,lineages,probes, 0)
    print "-------EVOLUTION WITH RECURRENT------"
    fitnessGenerationsOn = MainWithProbe(generations,lineages,probes, 1)
    print "-------EVOLUTION WITH RECURRENT AND MASK------"
    fitnessGenerationsMask = MainWithProbe(generations,lineages,probes, 2)

    plot = plt.plot(fitnessGenerationsWithout)
    plot = plt.plot(fitnessGenerationsOn)
    plot = plt.plot(fitnessGenerationsMask)
    plt.show(plot)

def fromFile(generations):
    MainFromFile(generations, 1, "../../RAGDOLL_DATA/bestParent.txt");

assign10(1000, 1, 100)
