from flask import Flask, render_template

app = Flask(__name__)

@app.route('/patient/<id>')
def patient(id):
    if id == "A_1":
        return """
        <html>
            <head>
                <title>A 병동 - 환자 정보</title>
                <style>
                    body {
                        background-color: #f44336;
                        color: white;
                        text-align: center;
                        font-family: Arial, sans-serif;
                    }
                    .card {
                        background: white;
                        color: black;
                        padding: 20px;
                        border-radius: 10px;
                        display: inline-block;
                        margin-top: 50px;
                    }
                </style>
            </head>
            <body>
                <h1>A 병동</h1>
                <div class='card'>
                    <h2>환자: 심승환</h2>
                    <p>병명: 고지혈증</p>
                </div>
            </body>
        </html>
        """
    else:
        return "<h1>환자 정보를 찾을 수 없습니다.</h1>"

if __name__ == "__main__":
    app.run(debug=True)
