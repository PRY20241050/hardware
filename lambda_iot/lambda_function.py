import json
import requests  # Importa el módulo requests para hacer peticiones HTTP

def lambda_handler(event, context):
    print("Received event: " + json.dumps(event))
    
    # Extrae los datos del evento
    device = 1
    date = event.get('date')
    time = event.get('time')
    pm25 = event.get('pm2.5')
    pm10 = event.get('pm10')
    co = event.get('co')
    no2 = event.get('no2')
    so2 = event.get('so2')
    temperature = event.get('temperature')
    
    pm25 = formatear_valores(pm25)
    pm10 = formatear_valores(pm10)
    co = formatear_valores(co)
    no2 = formatear_valores(no2)
    so2 = formatear_valores(so2)
    temperature = formatear_valores(temperature)
    # Define el endpoint al cual enviar los datos
    endpoint_url = 'https://iot-bend.vercel.app/api/measurements/create/'
    
    # Crea el payload con los datos que deseas enviar
    payload = {
        'deviceId':device,
        'date': date,
        'time': time,
        'pm25': pm25,
        'pm10': pm10,
        'co': co,
        'no2': no2,
        'so2': so2,
        'temperature': temperature
    }
    
    # Define los headers necesarios (en este caso, JSON)
    headers = {
        'Content-Type': 'application/json'
    }
    
    try:
        # Envía los datos al endpoint usando requests
        response = requests.post(endpoint_url, data=json.dumps(payload), headers=headers)

        if response.status_code == 200 or response.status_code == 201:
            print("Data sent to endpoint successfully")
        else:
            print(f"Failed to send data to endpoint. Status code: {response.status_code}")
        
    except Exception as e:
        print(f"Error: {str(e)}")
        return {
            'statusCode': 500,
            'body': json.dumps(f'Error: {str(e)}')
        }
        
    return {
        'statusCode': 200,
        'body': json.dumps('Event processed successfully')
    }
    
def formatear_valores(value):
    if isinstance(value, float):
        return '{:.3f}'.format(value)
    else:
        return value