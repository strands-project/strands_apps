#!/usr/bin/python

import smtplib
from email.MIMEMultipart import MIMEMultipart
from email.MIMEBase import MIMEBase
from email.MIMEText import MIMEText
from email import Encoders
import os

def sendmail(sender, pwd, to, subject, text, attach):
   msg = MIMEMultipart()

   msg['From'] = sender
   msg['To'] = ",".join(to)
   msg['Subject'] = subject

   msg.attach(MIMEText(text))

   part = MIMEBase('application', 'octet-stream')
   if attach!= "":
	   part.set_payload(open(attach, 'rb').read())
   	   Encoders.encode_base64(part)
	   part.add_header('Content-Disposition',
        	   'attachment; filename="%s"' % os.path.basename(attach))
	   msg.attach(part)

   mailServer = smtplib.SMTP("smtp.gmail.com", 587)
   mailServer.ehlo()
   mailServer.starttls()
   mailServer.ehlo()
   mailServer.login(sender, pwd)
   mailServer.sendmail(sender, to, msg.as_string())
   # Should be mailServer.quit(), but that crashes...
   mailServer.close()

