if (digitalRead(remotePin))
{
   analogWrite(ENABLE_PIN, 255); // Motorn körs i hög hastighet.
}
else
{
   analogWrite(ENABLE_PIN, 0); // annars är motorn av.
}

return;
}
