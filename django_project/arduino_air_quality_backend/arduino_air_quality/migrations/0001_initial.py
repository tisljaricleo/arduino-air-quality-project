# Generated by Django 3.2.1 on 2021-05-04 17:34

from django.db import migrations, models


class Migration(migrations.Migration):
    initial = True

    dependencies = [
    ]

    operations = [
        migrations.CreateModel(
            name='Sensor',
            fields=[
                ('id', models.BigAutoField(auto_created=True,
                                           primary_key=True, serialize=False,
                                           verbose_name='ID')),
                ('Name', models.CharField(max_length=50)),
                ('Type', models.CharField(max_length=50)),
            ],
        ),
    ]
