import json
import boto3

client = boto3.client('robomaker')

def lambda_handler(event, context):
    
    if (event['batchSimJobArn']):  
        
        # Set the default output. 
        output = { 'arns': None, 'isDone': False, 'batchSimJobArn': event['batchSimJobArn'], 'status': 'InProgress', 'codePipelineJobId': event['codePipelineJobId']}
        arns = []

        response = client.describe_simulation_job_batch(
            batch = event['batchSimJobArn']
        )
        
        if response['status'] == 'Completed':

            output['isDone'] = True
            
            # In this sample, we fail the code pipeline on any test failure. 
            if len(response['failedRequests']) == 0:
                output['status'] = 'Success'
            else:
                output['status'] = 'Failed'
                
            for job_output in response['createdRequests']:
                arns.append(job_output['arn'])
            
            output['arns'] = arns
            
        elif response['status'] == 'Failed' or response['status'] == 'Canceled':
            output['isDone'] = True
            output['status'] = 'Failed'
 
    else:
        output['isDone'] = True
        output['status'] = 'Failed'

    return output
