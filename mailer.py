from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from email.mime.application import MIMEApplication
from email.mime.multipart import MIMEMultipart
import smtplib
import os
import json

directory = os.getcwd()
personals = json.load(open('personal.json'))


def message(img=None):
    msg = MIMEMultipart()

    msg['Subject'] = "Fresher's Credentials"

    attachment = directory + "/TT.pdf"

    text = "Hello, How are you."

    msg.attach(MIMEText(text))

    if img is not None:
        if type(img) is not list:
            img = [img]

        for one_img in img:
            img_data = open(one_img, 'rb').read()
            msg.attach(MIMEImage(img_data,
                                 name=os.path.basename(one_img)))

    if attachment is not None:

        if type(attachment) is not list:
            attachment = [attachment]

        for one_attachment in attachment:
            with open(one_attachment, 'rb') as f:
                file = MIMEApplication(
                    f.read(),
                    name=os.path.basename(one_attachment)
                )
            file['Content-Disposition'] = f'attachment;\
                filename="{os.path.basename(one_attachment)}"'

            msg.attach(file)
    return msg


def mail(to_addr, img):
    smtp = smtplib.SMTP('smtp.gmail.com', 587)
    smtp.ehlo()
    smtp.starttls()

    email = personals["email"]
    password = personals["password"]
    smtp.login(email, password)

    msg = message(img)

    smtp.sendmail(from_addr="<noreply>.gmail.com", to_addrs=to_addr, msg=msg.as_string())
    smtp.quit()
