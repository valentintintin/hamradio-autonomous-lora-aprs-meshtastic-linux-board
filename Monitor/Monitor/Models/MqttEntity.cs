using System.Reactive.Linq;
using System.Reactive.Subjects;
using System.Text.Json;

namespace Monitor.Models;

public interface IStringConfigEntity
{
    string Id { get; }
    bool SaveInBdd { get; }
    bool SendToMqtt { get; }
    bool ChangeWithAck { get; }

    IObservable<string> ValueStringAsync();
    IObservable<string> ValueToChangeStringAsync();
    string ValueAsString();
    bool SetFromStringPayload(string? payload);
    void ClearValueToChange();
}

public class ConfigEntity<T> : IStringConfigEntity
{
    public string Id { get; }
    public bool SaveInBdd { get; }
    public bool SendToMqtt { get; }
    public bool ChangeWithAck { get; }

    public T? Value
    {
        get => ValuePrivate;
        set => SetValue(value);
    }
    private T? ValuePrivate { get; set; }
    private T? ValueToChangePrivate { get; set; }
    private T? OldValue { get; set; }
    
    private readonly Subject<(T? old, T? value)> _valueSubject = new();
    private readonly Subject<(T? value, T? valueToChange)> _valueToChangeSubject = new();
    private readonly T? _initialValue;

    public ConfigEntity(string id, bool saveInBdd = false, T? initialValue = default, bool sendToMqtt = false, bool changeWithAck = false)
    {
        SaveInBdd = saveInBdd;
        SendToMqtt = sendToMqtt;
        Id = id;
        ChangeWithAck = changeWithAck;
        OldValue = ValuePrivate = _initialValue = initialValue;
        SetValue(initialValue);
    }

    public IObservable<(T? old, T? value, string id)> ValueChanges(bool onlyIfDifferent = true)
    {
        return _valueSubject.AsObservable()
            .Where(v => !onlyIfDifferent || v.old?.Equals(v.value) != true)
            .Select(v => (v.old, v.value, Id));
    }

    public IObservable<(T? value, T? valueToChange, string id)> ValueToChange(bool onlyIfDifferent = true)
    {
        return _valueToChangeSubject.AsObservable()
            .Where(v => !onlyIfDifferent || v.value?.Equals(v.valueToChange) != true)
            .Select(v => (v.value, v.valueToChange, Id));
    }

    public IObservable<string> ValueStringAsync()
    {
        return _valueSubject.AsObservable().Select(v => JsonSerializer.Serialize(v.value));
    }

    public IObservable<string> ValueToChangeStringAsync()
    {
        return _valueToChangeSubject.AsObservable().Select(v => JsonSerializer.Serialize(v.valueToChange));
    }

    public string ValueAsString()
    {
        return JsonSerializer.Serialize(ValuePrivate);
    }

    public void SetValue(T? state, bool force = false)
    {
        if (ChangeWithAck && !force)
        {
            ValueToChangePrivate = state;
            _valueToChangeSubject.OnNext((ValuePrivate, ValueToChangePrivate));
            return;
        }
        
        OldValue = ValuePrivate;
        ValuePrivate = state;
        ValueToChangePrivate = default;
        
        _valueSubject.OnNext((OldValue, ValuePrivate));
    }

    public bool SetFromStringPayload(string? payload)
    {
        var newValue = string.IsNullOrWhiteSpace(payload) ? default : JsonSerializer.Deserialize<T?>(payload);

        if (newValue?.Equals(ValuePrivate) != true)
        {
            SetValue(newValue);

            return true;
        }

        return false;
    }

    public void SetValueToInitialValue()
    {
        Value = _initialValue;
    }

    public void ClearValueToChange()
    {
        ValueToChangePrivate = default;
    }

    public bool IsTrue()
    {
        return ValuePrivate?.ToString() == bool.TrueString;
    }

    public bool IsFalse()
    {
        return ValuePrivate?.ToString() == bool.FalseString;
    }

    public IObservable<bool> IsTrueAsync()
    {
        return ValueChanges().Select(v => v.value?.ToString() == bool.TrueString).Where(v => v);
    }

    public IObservable<bool> IsFalseAsync()
    {
        return ValueChanges().Select(v => v.value?.ToString() == bool.FalseString).Where(v => v);
    }

    public override string ToString()
    {
        return $"{Id} => {(ValueToChangePrivate != null ? $"({ValueToChangePrivate}) => " : "")}{Value}";
    }
}