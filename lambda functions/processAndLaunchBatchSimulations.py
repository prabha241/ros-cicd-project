import json
import time
import uuid
import os
import boto3
from copy import deepcopy

client = boto3.client('robomaker')
S3_BUCKET = 'ros-cicd-bucket'
IAM_ROLE = 'arn:aws:iam::498889106520:role/ec2-instance-builder'
SECURITY_GROUP = 'sg-04447ee21431760d5'
SUBNET_1 = 'subnet-05c69fd197cd4a9fd'
SUBNET_2 = 'subnet-0f03ec01d691f5807'
ROBOT_APP_ARN = 'arn:aws:robomaker:us-east-1:498889106520:robot-application/cicd-robot-app/1671307511567'
SIMULATION_APP_ARN = 'arn:aws:robomaker:us-east-1:498889106520:simulation-application/cicd-sim-app/1671307526373'

def lambda_handler(event, context):
    output = {
        'isDone': False,
        'codePipelineJobId': event['codePipelineJobId'],
        'batchSimJobArn': None
    }
    
    jobs = []

    for simulation in event['simulations']:
            
        print('Preparing simulation %s...' % json.dumps(simulation))

        if S3_BUCKET not in simulation.get('params', {}).get('outputLocation', {}).get('s3Bucket', {}):
            if not "outputLocation" in simulation['params']:
                simulation['params']['outputLocation'] = {}
            simulation['params']['outputLocation']['s3Bucket'] = S3_BUCKET
 
        if not "iamRole" in simulation['params']:
            simulation['params']['iamRole'] = IAM_ROLE
                    
        if 'vpcConfig' in simulation['params']:
            if "securityGroups" in simulation['params']['vpcConfig']:
                simulation['params']['vpcConfig']['securityGroups'].append(SECURITY_GROUP)
            if not 'subnets' in simulation['params']['vpcConfig']:
                simulation['params']['vpcConfig']['subnets'] = []
            if SUBNET_1 not in simulation['params']['vpcConfig']['subnets']:
                simulation['params']['vpcConfig']['subnets'].append(SUBNET_1)
            if SUBNET_2 not in simulation['params']['vpcConfig']['subnets']:
                simulation['params']['vpcConfig']['subnets'].append(SUBNET_2)

        
        for x, scenario in enumerate(simulation['scenarios']):
            
            if scenario in event['scenarios'].keys():
                
                _sim_params = deepcopy(simulation['params'])
                
                print('Scenario %s found...' % scenario)
        
                _sim_params['tags'] = { 'Scenario': scenario }
                y, z = 0, 0
        
                for y, robotApp in enumerate(_sim_params['robotApplications']):
                    _sim_params['robotApplications'][y]['launchConfig']['environmentVariables'] = event['scenarios'][scenario]['robotEnvironmentVariables']
                    if ROBOT_APP_ARN and not 'application' in _sim_params['robotApplications'][y]:
                        _sim_params['robotApplications'][y]['application'] = ROBOT_APP_ARN
                    
                for z, simApp in enumerate(_sim_params['simulationApplications']):
                    _sim_params['simulationApplications'][z]['launchConfig']['environmentVariables'] = event['scenarios'][scenario]['simEnvironmentVariables']
                    if SIMULATION_APP_ARN and not 'application' in _sim_params['simulationApplications'][z]:
                        _sim_params['simulationApplications'][z]['application'] = SIMULATION_APP_ARN
                
                print('Adding following job: ' + json.dumps(_sim_params))
                
                jobs.append(_sim_params)
                
            else:
                raise Exception('Scenario %s does not exist.' % scenario)
                
        response = client.start_simulation_job_batch(
            batchPolicy={
                'timeoutInSeconds': 800,
                'maxConcurrency': 1
            }, 
            createSimulationJobRequests=jobs, 
            tags = {
                'launcher': 'cicd_pipeline',
                'codePipelineJobId': event['codePipelineJobId']
        })

        output['batchSimJobArn'] = response['arn']
        
        if not output['batchSimJobArn']:
            raise Exception('Error launching batch simulation jobs. Check your scenarios JSON document.')
        
    return output
