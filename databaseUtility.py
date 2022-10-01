import mysql.connector as sql
import json

db = sql.connect(
    host="sql6.freesqldatabase.com",
    user="sql6514192",
    passwd="cPeZlsijGB",
    database="sql6514192",
    port="3306"
)


def execute(query):
    try:
        cursor = db.cursor()
        cursor.execute(query)
        res = cursor.fetchall()
        db.commit()
        return res

    except sql.Error as err:
        print(err)
        return -1


def fillDatabase():
    data = json.load(open("data.json"))

    for key, value in data.items():
        query = f"""
                INSERT INTO Student (rollno, shahash) VALUES ("{key}", "{value["hash"]}");
                """
        execute(query)


def clearDatabase():
    query = "DELETE FROM Student;"
    execute(query)


def getData():
    query = "SELECT * FROM Student;"
    res = execute(query)
    print(res)


if __name__ == "__main__":
    clearDatabase()
    fillDatabase()
    getData()

