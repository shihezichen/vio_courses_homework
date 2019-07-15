import base64

f = open('allan_01.png', 'rb')
code = base64.b64encode(f.read())
f.close()

f = open('base64.txt','wb')
f.write(code)
f.close()
print(code)