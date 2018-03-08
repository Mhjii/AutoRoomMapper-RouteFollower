import rethinkdb as DB

DB.connect("localhost",28015).repl()

cursor = DB.table("authors").run()

for document in cursor:
    print(document)

