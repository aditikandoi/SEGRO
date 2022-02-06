# Generated by Django 2.2.5 on 2020-01-28 13:17

from django.db import migrations, models


class Migration(migrations.Migration):

    initial = True

    dependencies = [
    ]

    operations = [
        migrations.CreateModel(
            name='Collector',
            fields=[
                ('id', models.AutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('name', models.CharField(max_length=50)),
                ('collector_id', models.IntegerField()),
                ('collector_location', models.CharField(max_length=50)),
                ('number_of_trips', models.IntegerField()),
            ],
        ),
    ]
