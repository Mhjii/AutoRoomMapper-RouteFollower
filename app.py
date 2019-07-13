from flask import Flask, render_template
import numpy
import matplotlib.pyplot as plt
from io import BytesIO
import base64


app = Flask(__name__)

@app.route('/')
def hello_world():
    ### Generating X,Y coordinaltes to be used in plot
    X = numpy.linspace(0,10,30)
    Y = X*X
    ### Generating The Plot
    plt.plot(X,Y)
    ### Saving plot to disk in png format


    ### Rendering Plot in Html
    figfile = BytesIO()
    plt.savefig(figfile, format='png')
    figfile.seek(0)
    figdata_png = base64.b64encode(figfile.getvalue())
    ### Remove b' from begining and ' in the end
    ### So that we can send the string within base64 noation
    result = str(figdata_png)[2:-1]
    return render_template('output.html', result=result)

hello_wor