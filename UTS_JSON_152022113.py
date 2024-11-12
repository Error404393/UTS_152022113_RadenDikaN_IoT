from datetime import datetime
from flask import Flask, request, send_file

app = Flask(__name__)

database = [
    {
        'idx': 101,
        'suhu': 36,
        'humid': 36,
        'kecerahan': 25,
        'timestamp': '2010-09-18 07:23:48'
    },
    {
        'idx': 226,
        'suhu': 36,
        'humid': 36,
        'kecerahan': 27,
        'timestamp': '2011-05-02 12:29:34'
    },
]

key = max(data['idx'] for data in database)

@app.get('/')
def index_html():
    return send_file('index.html')

@app.post('/api/post')
def post_data():
    global key
    json = request.get_json()

    print('tes')
    if not request.is_json or not json:
        return {
            'message': 'data is not json'
        }

    key += 1
    data = {
        'idx': key,
        'suhu': int(json['suhu']),
        'humid': int(json['kelembaban']),
        'kecerahan': int(json['kecerahan']),
        'timestamp': datetime.now().isoformat()
    }

    database.append(data)

    return {
        'message': 'success'
    }

@app.get('/api/get')
def get_data():
    data_suhu = [data['suhu'] for data in database]
    month_year_max = [
        {
            'month_year': datetime.fromisoformat(data['timestamp']).strftime('%m-%Y')
        } for data in database
    ]

    suhumax = max(data_suhu)
    suhumin = min(data_suhu)
    suhurata = sum(data_suhu) / len(data_suhu)

    data = {
        'suhumax': suhumax,
        'suhumin': suhumin,
        'suhurata': suhurata,
        'nilai_suhu_max_humid_max': database,
        'month_year_max': month_year_max
    }

    return data

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=9990, debug=True)
