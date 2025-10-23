class PushNotification {
    private:
        system_tick_t _lastSent = 0;
        system_tick_t _lastSentDebugMessage = 0;
        bool _debugRateLimitWarningSent = false;
        //system_tick_t _lastAttemptDuringCooldown = 0;
        std::chrono::milliseconds _cooldownTimeMinutes;
        particle::Future<bool> _publishFuture;
        particle::Future<bool> _publishFutureDebug;
        
    public:
        PushNotification(std::chrono::milliseconds cooldownTimeMinutes=5min) {
            _cooldownTimeMinutes = cooldownTimeMinutes;
        }
        
        ~PushNotification() {
        }
        
        static particle::Future<bool> send(const char* message, bool criticalAlert=false) {
            return Particle.publish("push-notification", String::format("{ \"type\": \"send-message-%s\", \"message\": \"[Aquarium controller] %s""\" }", criticalAlert ? "critical" : "generic", message), PRIVATE | WITH_ACK);
        }

        // note: this method will delay by 1000ms if it's called in more than once a second
        // TODO: consider using hashes to skip delaying / publishing, e.g. std::size_t h1 = std::hash<std::string>{}(message);
        bool sendWithCooldown(const char* message, bool criticalAlert=false) {

            if (_lastSent == 0 || (millis() - _lastSent > _cooldownTimeMinutes.count())) {
                _publishFuture = send(message, criticalAlert); //(const char* message, bool criticalAlert=false, PublishFlags publishFlags=PRIVATE)
                _lastSent = millis();
                return true;
            } else {
                /*
                // for testing, add this to top of loop()
                static PushNotification notification_test(15s);
                notification_test.sendWithCooldown("Testing...");
                return;
                */
                if (_lastSentDebugMessage == 0 || (millis() - _lastSentDebugMessage >= 60*1000)) {
                    _publishFutureDebug = Particle.publish("debug", String::format("Skipping notification; in cooldown period. Message: \"%s\"", String(message)));
                    _lastSentDebugMessage = millis();
                    delay(1000); // delay so we're not adding to our publish() rate limit
                } else if (!_debugRateLimitWarningSent) {
                    // skip further notifications so we don't exhaust our publish() rate limit
                    Particle.publish("debug", String::format("Warning: Skipping further debug notification to protect publish() rate limits. Message: \"%s\"", String(message))); // was getting gibberish for message string earlier
                    delay(1000);
                    _debugRateLimitWarningSent = true;
                }

                /*
                _publishFutureDebug = Particle.publish("debug", String::format("Skipping notification; in cooldown period. Message: \"%s\"", message));
                if (_lastAttemptDuringCooldown == 0 || (millis() - _lastAttemptDuringCooldown <= 1000)) {
                    delay(1000); // ensure we don't get rate-limited due to too many debug messages
                }
                _lastAttemptDuringCooldown = millis();
                */
            }
            return false;
        }
};