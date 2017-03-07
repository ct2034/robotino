import base64
import gzip

import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, MaxNLocator


def enc(s):
    labels = []

    labels.append('s')
    # print(s)
    # print("l: ", s.__len__())

    # print("..")
    b = str.encode(s, 'ascii')
    labels.append('b')
    # print(b)
    # print("l: ", b.__len__())

    # print("..")

    b64 = base64.standard_b64encode(b)
    labels.append('b64')
    # print(b64)
    # print("l: ", b64.__len__())

    # print("..")

    z = gzip.compress(b)
    labels.append('z')
    # print(z)
    # print("l: ", z.__len__())

    # print("..")
    z64 = base64.standard_b64encode(z)
    labels.append('z64')
    # print(z64)
    # print("l: ", z64.__len__())

    # print("..")
    z85 = base64.b85encode(z)
    labels.append('z85')
    # print(z85)
    # print("l: ", z85.__len__())

    return (
        (
            s.__len__(),
            b.__len__(),
            b64.__len__(),
            z.__len__(),
            z64.__len__(),
            z85.__len__()
        ),
        labels
    )


print("..")

strings = [
    'aaaaaaaaaaaaaaaaaaaaaaa',
    'hi!',
    '{"glossary":{"title":"exampleglossary","GlossDiv":{"title":"S","GlossList":{"GlossEntry":{"ID":"SGML","SortAs":"SGML","GlossTerm":"StandardGeneralizedMarkupLanguage","Acronym":"SGML","Abbrev":"ISO8879:1986","GlossDef":{"para":"Ameta-markuplanguage,usedtocreatemarkuplanguagessuchasDocBook.","GlossSeeAlso":["GML","XML"]},"GlossSee":"markup"}}}}}',
    'Lorem rutrum vestibulum metus. Non ac iaculis diam, a pede. Accumsan, mi suspendisse duis, ridiculus pharetra nulla conubia amet. Enim mollis at mus eu. Et, litora inceptos vestibulum mauris sed, amet mus a montes neque neque orci. Mauris, ad, enim mattis urna, habitasse sapien elementum eleifend taciti tempor leo at platea. Augue ve hymenaeos eu, ut mollis in, magnis per, euismod. Justo aliquet ac ve semper per, vestibulum proin orci ac sollicitudin lacinia. Quam urna. Duis neque.',
    '   |00|01|02|03|04|05|06|07|08|09|10|11|12|13|14|15|16|17|18|19|20|21|22|23|24|25|26|27|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|..|..|..|..|..|..|..|..|..|..|..|..|@@|@@|..|..|..|..|..|..|..|..|..|..|..|..|@@|@@|..|@@|@@|@@|@@|..|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|..|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|..|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|..|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|..|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|..|@@|@@|@@|@@|..|@@|@@|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|@@|@@|..|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|..|@@|@@|..|..|..|..|..|..|@@|@@|..|..|..|..|@@|@@|..|..|..|..|@@|@@|..|..|..|..|..|..|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|..|..|..|..|..|..|..|..|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|..|..|..|..|..|..|..|..|..|..|@@|@@|@@|@@|@@|@@|@@|@@|..|..|..|..|..|..|..|..|..|..|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|..|..|..|..|..|..|..|..|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|..|..|..|..|..|..|..|..|..|..|..|..|@@|@@|..|..|..|..|..|..|..|..|..|..|..|..|@@|@@|..|@@|@@|@@|@@|..|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|..|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|..|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|..|@@|@@|@@|@@|..|@@|@@|..|..|..|@@|@@|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|@@|@@|..|..|..|@@|@@|@@|@@|..|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|..|@@|@@|@@|@@|..|..|..|..|..|..|@@|@@|..|..|..|..|@@|@@|..|..|..|..|@@|@@|..|..|..|..|..|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|..|@@|@@|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|..|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|@@|',
    '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'
]

res = []
width = 30

fig = plt.figure()
ax = fig.add_subplot(111)

for s in strings:
    if s.__len__() > width:
        print(s[0:width - 3], " ...")
    else:
        print(s)
    (ys, labels) = enc(s)
    print(ys)

    xs = range(len(ys))

    def format_fn(tick_val, tick_pos):
        if int(tick_val) in xs:
            return labels[int(tick_val)]
        else:
            return ''

    ax.xaxis.set_major_formatter(FuncFormatter(format_fn))
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))

    ax.plot(xs, ys)
    plt.yscale('log')

    print("--")

plt.show()
