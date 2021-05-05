from django.db import models

# Create your models here.
class Sensor(models.Model):
    Name = models.CharField(max_length=50)
    Type = models.CharField(max_length=50)