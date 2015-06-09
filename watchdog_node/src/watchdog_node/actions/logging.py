from . import ActionType  
import smtplib
import rospy

class SendEmail(ActionType):
    name = "SendEmail"
    description = "Sends an email using simple unsecure SMTP."
    config_keys = [("to_addresses", "A list of email addresses to send to."),
                   ("from_address", "The email address to be sent from."),
                   ("message", "The message body of the email."),
                   ("server", "The SMTP server."),
                   ("port", "The port the SMTP server uses.")]
    
    def execute(self):
        try:
            msg = ("From: %s\r\nTo: %s\r\n\r\n%s"
                   % (self.from_address, ", ".join(self.to_addresses),
                      self.message))
            
            server = smtplib.SMTP(self.server, self.port)
            server.set_debuglevel(False)
            server.sendmail(self.from_address, self.to_addresses, msg)
            server.quit()
        except Exception, e:
            rospy.logerr("Could not send email: {}".format(e))
    
